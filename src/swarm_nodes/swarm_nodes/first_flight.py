"""Own the flight sequence: GUIDED -> arm -> takeoff -> hand over to the bridge -> land.

The command bridge deliberately knows nothing about arming or modes; this node does, so there is
one place that can put a drone in the air and one place that brings it down.

Safety behaviour, in order of how much it matters:

  * Ctrl-C lands every drone. It must never mean "abandon armed aircraft in GUIDED".
  * It refuses to arm unless confirm:=true is passed explicitly.
  * It counts down arm_delay seconds before arming so there is time to abort.
  * It waits for every drone to report connected, and refuses to arm a partial swarm.

Usage:

    ros2 run swarm_nodes first_flight --ros-args \\
        -p confirm:=true -p namespaces:="['drone1']" -p takeoff_altitude:=1.0

Run the command bridge alongside it. Once this node logs the handover, /droneN/cmd_vel is live.
"""
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

# Set by our own SIGINT handler. rclpy's default handler tears the context down immediately, which
# would leave land_all() unable to call any service -- Ctrl-C would abandon armed aircraft instead
# of landing them. We disable it in main() and shut down ourselves, after landing.
_interrupted = False


def _on_signal(signum, frame):
    global _interrupted
    _interrupted = True


def running():
    """True while we should keep waiting. Goes False the moment Ctrl-C is pressed, so every
    waiting loop below drops out immediately instead of sitting out its full timeout."""
    return rclpy.ok() and not _interrupted


class FirstFlight(Node):
    def __init__(self):
        super().__init__('first_flight')
        self.drone_ns = self.declare_parameter('namespaces', ['drone1']).value
        self.altitude = self.declare_parameter('takeoff_altitude', 1.0).value
        self.confirm = self.declare_parameter('confirm', False).value
        self.arm_delay = self.declare_parameter('arm_delay', 10.0).value
        self.connect_timeout = self.declare_parameter('connect_timeout', 30.0).value

        self.state = {ns: None for ns in self.drone_ns}
        self.armed = set()

        self.mode_cli, self.arm_cli, self.takeoff_cli, self.land_cli = {}, {}, {}, {}
        for ns in self.drone_ns:
            self.create_subscription(State, f'/{ns}/mavros/state',
                                     lambda msg, ns=ns: self.on_state(ns, msg),
                                     qos_profile_sensor_data)
            self.mode_cli[ns] = self.create_client(SetMode, f'/{ns}/mavros/set_mode')
            self.arm_cli[ns] = self.create_client(CommandBool, f'/{ns}/mavros/cmd/arming')
            self.takeoff_cli[ns] = self.create_client(CommandTOL, f'/{ns}/mavros/cmd/takeoff')
            self.land_cli[ns] = self.create_client(CommandTOL, f'/{ns}/mavros/cmd/land')

    def on_state(self, ns, msg):
        self.state[ns] = msg

    # -- helpers ---------------------------------------------------------------

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and running():
            rclpy.spin_once(self, timeout_sec=0.1)

    def call(self, client, request, ns, what, timeout=10.0, during_shutdown=False):
        """Call a service and wait. Returns the response, or None on failure.

        during_shutdown keeps spinning after a Ctrl-C, which land_all() needs: every other wait
        aborts on the interrupt, but the landing call is the one that must still go through.
        """
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{ns}: {what} service unavailable')
            return None
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while (during_shutdown or running()) and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            self.get_logger().error(f'{ns}: {what} timed out')
            return None
        return future.result()

    # -- sequence --------------------------------------------------------------

    def wait_connected(self):
        self.get_logger().info(f'waiting for {len(self.drone_ns)} drone(s) to connect...')
        end = time.monotonic() + self.connect_timeout
        while running() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            missing = [ns for ns in self.drone_ns
                       if self.state[ns] is None or not self.state[ns].connected]
            if not missing:
                self.get_logger().info('all drones connected')
                return True
        missing = [ns for ns in self.drone_ns
                   if self.state[ns] is None or not self.state[ns].connected]
        self.get_logger().error(f'not connected: {", ".join(missing)} -- refusing to arm')
        return False

    def set_guided(self):
        ok = True
        for ns in self.drone_ns:
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            res = self.call(self.mode_cli[ns], req, ns, 'set_mode')
            if res is None or not res.mode_sent:
                self.get_logger().error(f'{ns}: could not enter GUIDED')
                ok = False
            else:
                self.get_logger().info(f'{ns}: GUIDED')
        return ok

    def countdown(self):
        """Shout for a few seconds before arming, so there is time to hit Ctrl-C."""
        self.get_logger().warn(
            f'ARMING {len(self.drone_ns)} DRONE(S) in {self.arm_delay:.0f}s -- Ctrl-C to abort')
        remaining = self.arm_delay
        while remaining > 0 and running():
            self.get_logger().warn(f'  {remaining:.0f}...')
            self.spin_for(min(1.0, remaining))
            remaining -= 1.0

    def arm_all(self):
        ok = True
        for ns in self.drone_ns:
            req = CommandBool.Request()
            req.value = True
            res = self.call(self.arm_cli[ns], req, ns, 'arming')
            if res is None or not res.success:
                self.get_logger().error(f'{ns}: ARMING REFUSED (check pre-arm messages)')
                ok = False
            else:
                self.armed.add(ns)
                self.get_logger().info(f'{ns}: armed')
        return ok

    def takeoff_all(self):
        for ns in self.drone_ns:
            req = CommandTOL.Request()
            req.altitude = float(self.altitude)
            res = self.call(self.takeoff_cli[ns], req, ns, 'takeoff')
            if res is None or not res.success:
                self.get_logger().error(f'{ns}: takeoff refused')
            else:
                self.get_logger().info(f'{ns}: taking off to {self.altitude} m')
        # Rough wait for the climb: about 3 seconds per metre, never less than 5.
        self.spin_for(max(5.0, self.altitude * 3.0))

    def land_all(self):
        """Called on the way out, including on Ctrl-C. Best effort, never raises."""
        if not self.armed:
            return
        self.get_logger().warn(f'landing {len(self.armed)} drone(s)')
        for ns in sorted(self.armed):
            try:
                req = CommandTOL.Request()
                res = self.call(self.land_cli[ns], req, ns, 'land', timeout=5.0,
                                during_shutdown=True)
                if res is None or not res.success:
                    self.get_logger().error(
                        f'{ns}: LAND REFUSED -- take manual control with the RC now')
                else:
                    self.get_logger().info(f'{ns}: landing')
            except Exception as err:                      # noqa: BLE001 - must not escape
                self.get_logger().error(f'{ns}: land failed: {err}')

    def run(self):
        if not self.confirm:
            self.get_logger().error(
                'refusing to arm without confirm:=true. Re-run with '
                '--ros-args -p confirm:=true once the area is clear and a safety pilot is ready.')
            return
        if not self.wait_connected():
            return
        if not self.set_guided():
            self.get_logger().error('aborting: not all drones reached GUIDED')
            return
        self.countdown()
        if not self.arm_all():
            self.get_logger().error('aborting: not all drones armed')
            self.land_all()
            return
        self.takeoff_all()
        self.get_logger().info(
            'HANDOVER: the command bridge now has control. Publish to /droneN/cmd_vel. '
            'Ctrl-C here to land.')
        while running():
            rclpy.spin_once(self, timeout_sec=0.2)


def main():
    # Take Ctrl-C over from rclpy. Its own handler shuts ROS down instantly, which would leave
    # land_all() unable to call anything -- Ctrl-C would abandon armed drones in mid-air instead
    # of bringing them down. We set a flag instead, land, and only then shut down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = FirstFlight()
    try:
        node.run()
    finally:
        # The whole point: Ctrl-C brings them down instead of leaving them hovering.
        node.land_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
