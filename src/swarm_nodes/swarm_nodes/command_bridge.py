"""Validate, clamp and watchdog every velocity command before it reaches the autopilot.

Subscribes /droneN/cmd_vel (what the formation controller and teleop_velocity publish) and
forwards it to /droneN/mavros/setpoint_velocity/cmd_vel_unstamped. Nothing else in the stack is
allowed to command a drone directly -- this is the single place where speed limits and the
loss-of-command behaviour live.

A fixed-rate timer is correct here, unlike in odom_relay. ArduPilot halts a vehicle if setpoints
stop arriving, so the bridge has to keep sending even when the controller goes quiet. The rule is
the mirror image of the telemetry one: never synthesize telemetry, always synthesize commands.

What happens when commands stop arriving:

    age < stale_timeout     forward the clamped command
    age >= stale_timeout    publish zero velocity -- stop and hold

Normally that is the end of it. The drone stays in GUIDED holding position, and the moment the
controller comes back it simply carries on, with no handover to redo.

ArduPilot has its own backstop underneath ours. After GUID_TIMEOUT seconds (a parameter, default
3.0) with no new setpoint it zeroes the target itself and holds position AUTONOMOUSLY -- needing
nothing from the ground station and nothing from a pilot. So if this node dies too, the drone still
stops and holds. Our 0.5 s does the same thing 2.5 s sooner, which at 1 m/s is 2.5 m of
uncommanded travel avoided. That is the whole value of the stale timeout, and it is real.

Switching flight mode is OFF by default, because it is not an improvement on the above. It was in
the original design and that was a mistake: taking the drone out of GUIDED moves it off a hold that
already works by itself. Set hold_mode if you specifically want a deliberate re-entry to GUIDED
before anything can command the drone again -- but know what each option costs:

    BRAKE   holds position and altitude by itself, ignores the sticks. The safe choice if you
            want the barrier.
    LOITER  holds position, but takes CLIMB RATE from the pilot's throttle stick. With nobody on
            the sticks it descends and lands, which is worse than simply staying in GUIDED.

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

        # Both timeouts are in SECONDS.
        #
        # stale_timeout -- how long we may keep flying on an out-of-date command. Short, because
        # a stale velocity is a drone moving with nobody steering it.
        self.stale_timeout = self.declare_parameter('stale_timeout', 0.5).value

        # hold_mode -- which flight mode to hand the drone to once commands have stopped for
        # hold_timeout seconds. EMPTY BY DEFAULT, meaning we never change mode: we just keep
        # streaming zeros, the drone holds in GUIDED, and if this node dies ArduPilot's own
        # GUID_TIMEOUT (default 3 s) takes over and holds it autonomously.
        #
        # Set it only if you want to force a deliberate re-entry to GUIDED afterwards:
        #   'BRAKE'   holds by itself, ignores the sticks        <- use this one
        #   'LOITER'  takes climb rate from the pilot's throttle; with nobody holding it,
        #             the drone descends and lands
        self.hold_mode = self.declare_parameter('hold_mode', '').value
        self.hold_timeout = self.declare_parameter('hold_timeout', 2.0).value

        self.max_xy = self.declare_parameter('max_xy_speed', 1.0).value
        self.max_z = self.declare_parameter('max_z_speed', 0.5).value
        self.max_yaw = self.declare_parameter('max_yaw_rate', 0.5).value

        self.last_cmd = {ns: None for ns in self.drone_ns}      # Twist, already clamped
        self.last_time = {ns: None for ns in self.drone_ns}     # seconds, monotonic-ish
        self.handed_over = {ns: False for ns in self.drone_ns}  # mode switched, awaiting GUIDED
        self.handover_sent = {ns: False for ns in self.drone_ns}  # request in flight, don't spam
        self.mode = {ns: '' for ns in self.drone_ns}

        self.setpoint_pubs = {}
        self.mode_clients = {}
        for ns in self.drone_ns:
            self.setpoint_pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
            # A "client" is a phone line to a service on another node. Creating it does not do
            # anything yet -- it just sets up our ability to later ask MAVROS "put this drone in
            # mode X". hand_over() below is what actually makes the call, and only
            # if hold_mode was set -- by default we never change mode at all.
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
            f'zero velocity after {self.stale_timeout}s, then '
            + (f'{self.hold_mode} after {self.hold_timeout}s'
               if self.hold_mode else
               "hold in GUIDED (ArduPilot's GUID_TIMEOUT is the backstop)"))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, ns, msg):
        # Returning to GUIDED is how the operator hands control back after a mode switch.
        if self.handed_over[ns] and msg.mode == 'GUIDED':
            self.handed_over[ns] = False
            self.handover_sent[ns] = False
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
            if last is None or self.handed_over[ns]:
                # Never been commanded (so stay quiet and let the mission node take off in
                # peace), or already tripped and waiting to be put back in GUIDED.
                continue

            # How many seconds since the last command arrived?
            age = self.now() - last

            if age < self.stale_timeout:
                self.setpoint_pubs[ns].publish(self.last_cmd[ns])   # fresh -- fly it
            elif not self.hold_mode or age < self.hold_timeout:
                # Stop and hold. With no hold_mode set this is where we stay: the drone holds in
                # GUIDED, recoverable the instant commands resume.
                self.setpoint_pubs[ns].publish(ZERO)
            else:
                self.hand_over(ns, age)                             # opt-in mode switch

    def hand_over(self, ns, age):
        """Opt-in: take the drone out of GUIDED so it cannot be commanded without a deliberate
        re-entry. Off unless hold_mode is set.

        We only stop streaming once the mode change is CONFIRMED. If the request fails or is
        refused we keep sending zeros, because going silent on a drone still in GUIDED would
        leave it on nothing but ArduPilot's own timeout.
        """
        if self.handover_sent[ns]:
            return                                  # request already in flight, don't spam at 20 Hz
        client = self.mode_clients[ns]
        if not client.service_is_ready():
            self.get_logger().error(
                f'{ns}: set_mode unavailable, cannot switch to {self.hold_mode}; '
                f'holding on zero velocity instead', throttle_duration_sec=10.0)
            return                                  # stream() keeps publishing zeros
        self.handover_sent[ns] = True
        self.get_logger().warn(
            f'{ns}: no command for {age:.1f}s, switching to {self.hold_mode}')
        req = SetMode.Request()
        req.custom_mode = self.hold_mode
        future = client.call_async(req)
        future.add_done_callback(lambda f, ns=ns: self.on_mode_set(ns, f))

    def on_mode_set(self, ns, future):
        self.handover_sent[ns] = False
        result = future.result()
        if result is None or not result.mode_sent:
            self.get_logger().error(
                f'{ns}: {self.hold_mode} was refused; staying on zero velocity')
        else:
            self.handed_over[ns] = True             # only now is it safe to stop streaming
            self.get_logger().info(f'{ns}: now in {self.hold_mode}, send GUIDED to resume')


def main():
    rclpy.init()
    rclpy.spin(CommandBridge())


if __name__ == '__main__':
    main()
