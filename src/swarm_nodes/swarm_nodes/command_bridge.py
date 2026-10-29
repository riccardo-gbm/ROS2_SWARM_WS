"""Validate, clamp and watchdog every velocity command before it reaches the autopilot.

Subscribes /droneN/cmd_vel (what the formation controller and teleop_velocity publish) and
forwards it to /droneN/mavros/setpoint_velocity/cmd_vel_unstamped. Nothing else in the stack is
allowed to command a drone directly -- this is the single place where speed limits and the
loss-of-command behaviour live.

A fixed-rate timer is correct here, unlike in odom_relay. ArduPilot stops a vehicle after 3 s
without a new setpoint and expects at least 1 Hz, so the bridge has to keep sending even when the
controller goes quiet. The rule is the mirror image of the telemetry one: never synthesize
telemetry, always synthesize commands.

What happens when commands stop arriving:

    age < stale_timeout      forward the clamped command
    age < loiter_timeout     publish zero velocity -- hold position
    age >= loiter_timeout    switch to the hold mode once, then stop streaming

The hold mode is the loiter_mode parameter, LOITER by default. LOITER takes throttle from the RC,
so without a pilot on the sticks the drone descends and lands; pass loiter_mode:=BRAKE for a hold
that needs no pilot input.

After a hold trip the bridge stays silent until the drone is put back into GUIDED, so regaining
control is always a deliberate act. It also stays silent before the very first command, so it can
never fight the mission node during arm and takeoff.

Twist is in world ENU (x = east, y = north, z = up), angular.z is yaw rate. MAVROS converts to NED.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

ZERO = Twist()


class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        # How often we send to the drone, in times per second.
        stream_hz = self.declare_parameter('stream_hz', 20.0).value

        # Two timeouts, both in SECONDS, because a hiccup and a crash deserve different answers.
        #
        #   0.5 s -- the controller skipped a few messages, or the network blipped.
        #            Stop moving and wait. We stay in GUIDED, so the moment commands come back
        #            the drone just keeps flying. Nothing to re-do.
        #
        #   2.0 s -- something is genuinely broken (controller crashed, laptop died).
        #            Stop relying on the ground station and hand the drone to a holding mode.
        #
        # One timeout would force a bad choice: too short and every blip kicks you out of GUIDED,
        # too long and a real crash leaves the drone coasting on an old command.
        self.stale_timeout = self.declare_parameter('stale_timeout', 0.5).value
        self.loiter_timeout = self.declare_parameter('loiter_timeout', 2.0).value

        # Which mode to hand the drone to when commands stop for good.
        #
        #   LOITER holds position BUT takes throttle from the pilot's RC stick:
        #     - safety pilot holding mid-throttle -> it hovers       <- the normal field case
        #     - nobody on the sticks              -> it descends and lands
        #       (that is what happened in simulation, where there is no transmitter at all)
        #   BRAKE  holds position entirely on its own and ignores the sticks.
        self.hold_mode = self.declare_parameter('loiter_mode', 'LOITER').value
        self.max_xy = self.declare_parameter('max_xy_speed', 1.0).value
        self.max_z = self.declare_parameter('max_z_speed', 0.5).value
        self.max_yaw = self.declare_parameter('max_yaw_rate', 0.5).value

        self.last_cmd = {ns: None for ns in self.drone_ns}      # Twist, already clamped
        self.last_time = {ns: None for ns in self.drone_ns}     # seconds, monotonic-ish
        self.loitered = {ns: False for ns in self.drone_ns}     # tripped, awaiting re-handover
        self.mode = {ns: '' for ns in self.drone_ns}

        self.setpoint_pubs = {}
        self.mode_clients = {}
        for ns in self.drone_ns:
            self.setpoint_pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
            # A "client" is a phone line to a service on another node. Creating it does not do
            # anything yet -- it just sets up our ability to later ask MAVROS "put this drone in
            # mode X". trip_loiter() below is what actually makes the call.
            # (A topic is a broadcast: shout and move on. A service is a request: ask, get a reply.)
            self.mode_clients[ns] = self.create_client(SetMode, f'/{ns}/mavros/set_mode')
            self.create_subscription(Twist, f'/{ns}/cmd_vel',
                                     lambda msg, ns=ns: self.on_cmd(ns, msg), 10)
            self.create_subscription(State, f'/{ns}/mavros/state',
                                     lambda msg, ns=ns: self.on_state(ns, msg),
                                     qos_profile_sensor_data)

        self.create_timer(1.0 / stream_hz, self.stream)
        self.get_logger().info(
            f'bridging {len(self.drone_ns)} drones at {stream_hz:.0f} Hz | '
            f'limits {self.max_xy} m/s xy, {self.max_z} m/s z, {self.max_yaw} rad/s yaw | '
            f'zero velocity after {self.stale_timeout}s, '
            f'{self.hold_mode} after {self.loiter_timeout}s')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, ns, msg):
        # Returning to GUIDED is how the operator hands control back after a LOITER trip.
        if self.loitered[ns] and msg.mode == 'GUIDED':
            self.loitered[ns] = False
            self.get_logger().info(f'{ns}: back in GUIDED, bridge re-armed')
        self.mode[ns] = msg.mode

    def on_cmd(self, ns, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn(f'{ns}: command contains NaN or inf, ignored',
                                   throttle_duration_sec=2.0)
            return
        self.last_cmd[ns] = self.clamp(msg)
        self.last_time[ns] = self.now()

    def clamp(self, msg):
        """Squeeze a requested velocity down to something safe."""
        out = Twist()

        # Sideways (east/north): shrink the whole arrow, never x and y separately.
        # Trimming each axis on its own would also TURN the command. Asking for (3, 4) -- which
        # points north-east -- would come out as (1, 1), which points 45 degrees off course, at
        # full allowed speed. So we work out the total speed and scale both numbers by the same
        # factor, which keeps the direction and only shortens the arrow.
        vx, vy = msg.linear.x, msg.linear.y
        speed = math.hypot(vx, vy)                 # total speed, ignoring direction
        if speed > self.max_xy:
            scale = self.max_xy / speed            # e.g. 1.0 / 5.0 = shrink to a fifth
            vx, vy = vx * scale, vy * scale
        out.linear.x, out.linear.y = vx, vy

        # Up/down and spin do not affect each other or the sideways direction, so each can simply
        # be squeezed into its own range. Python has no built-in "clamp", so this is the usual
        # sandwich -- read it inside out:
        #     min(limit, value)       caps the top
        #     max(-limit, that)       caps the bottom
        # With max_z = 0.5:   9.0 -> 0.5     -9.0 -> -0.5     0.3 -> 0.3 (left alone)
        out.linear.z = max(-self.max_z, min(self.max_z, msg.linear.z))
        out.angular.z = max(-self.max_yaw, min(self.max_yaw, msg.angular.z))
        return out

    def stream(self):
        """Runs 20 times a second, forever, whether or not anyone is sending us commands.

        That is the point. ArduPilot halts the drone if setpoints stop arriving, so we keep
        something flowing and decide for ourselves what that something should be.
        """
        for ns in self.drone_ns:
            last = self.last_time[ns]
            if last is None or self.loitered[ns]:
                # Never been commanded (so stay quiet and let the mission node take off in
                # peace), or already tripped and waiting to be put back in GUIDED.
                continue

            # How many seconds since the last command arrived?
            age = self.now() - last

            if age < self.stale_timeout:
                self.setpoint_pubs[ns].publish(self.last_cmd[ns])   # fresh -- fly it
            elif age < self.loiter_timeout:
                self.setpoint_pubs[ns].publish(ZERO)                # quiet briefly -- stop, wait
            else:
                self.trip_loiter(ns, age)                           # quiet too long -- hand over

    def trip_loiter(self, ns, age):
        self.loitered[ns] = True
        self.get_logger().warn(
            f'{ns}: no command for {age:.1f}s, switching to {self.hold_mode}')
        client = self.mode_clients[ns]
        if not client.service_is_ready():
            self.get_logger().error(
                f'{ns}: set_mode service unavailable, could NOT switch to {self.hold_mode}')
            return
        req = SetMode.Request()
        req.custom_mode = self.hold_mode
        future = client.call_async(req)
        future.add_done_callback(lambda f, ns=ns: self.on_mode_set(ns, f))

    def on_mode_set(self, ns, future):
        result = future.result()
        if result is None or not result.mode_sent:
            self.get_logger().error(f'{ns}: {self.hold_mode} request was not accepted')
        else:
            self.get_logger().info(f'{ns}: now in {self.hold_mode}, send GUIDED to resume')


def main():
    rclpy.init()
    rclpy.spin(CommandBridge())


if __name__ == '__main__':
    main()
