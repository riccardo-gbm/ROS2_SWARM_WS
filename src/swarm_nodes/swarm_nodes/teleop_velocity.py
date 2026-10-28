"""Keyboard velocity teleop for one drone, for bench and SITL testing.

Publishes a TwistStamped to <drone_id>/mavros/setpoint_velocity/cmd_vel while a key is held.

    w / s   +x / -x        a / d   -y / +y        q / e   -z / +z
    left / right arrow     yaw rate              Ctrl-C  quit

AXES ARE WORLD ENU, NOT BODY. MAVROS interprets this twist in the world frame regardless of
header.frame_id, so +x is EAST and +y is NORTH -- "w" does not mean "forward" and does not depend on
where the drone is pointing.

SAFETY: this goes straight to MAVROS with no speed clamp and no watchdog, and it is not the velocity
interface the formation controller will use. Run it in SITL first, and on a real aircraft only with
an RC safety pilot on the sticks and the geofence enabled.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistStamped

INPUT_POLL_HZ = 100.0


class TeleopVelocity(Node):
    def __init__(self):
        super().__init__('teleop_velocity')
        drone_id = self.declare_parameter('drone_id', '').value
        self.speed = self.declare_parameter('speed', 0.5).value
        self.yaw_rate = self.declare_parameter('yaw_rate', 0.5).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.hold_timeout = self.declare_parameter('hold_timeout', 0.5).value

        if not drone_id:
            raise ValueError(
                'drone_id is required, e.g. --ros-args -p drone_id:=drone1')
        drone_id = '/' + drone_id.strip('/')

        self.msg = TwistStamped()
        self.last_key = 0.0

        topic = f'{drone_id}/mavros/setpoint_velocity/cmd_vel'
        self.publisher = self.create_publisher(
            TwistStamped, topic, qos_profile_system_default)

        self.create_timer(1.0 / INPUT_POLL_HZ, self.read_key)
        self.create_timer(1.0 / rate_hz, self.publish_command)

        self.get_logger().info(
            f'{topic} at {rate_hz:.0f} Hz, {self.speed} m/s, {self.yaw_rate} rad/s')
        self.get_logger().warn(
            'axes are world ENU: w = EAST, a/d = north/south, NOT body-relative. '
            'No clamp and no watchdog -- SITL or RC safety pilot only.')

    def stop(self):
        self.msg.twist.linear.x = 0.0
        self.msg.twist.linear.y = 0.0
        self.msg.twist.linear.z = 0.0
        self.msg.twist.angular.z = 0.0

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            # Terminal key-repeat is far slower than this timer, so hold the last command briefly
            # instead of zeroing between repeats, which would make a held key stutter.
            if time.monotonic() - self.last_key > self.hold_timeout:
                self.stop()
            return

        key = sys.stdin.read(1)
        if key == '\x1b':  # escape sequence: arrow keys arrive as ESC [ A-D
            key = (key + sys.stdin.read(2))[-1]
        if key == '\x03':  # Ctrl-C, delivered as a byte because the terminal is raw
            raise KeyboardInterrupt

        self.last_key = time.monotonic()
        self.stop()
        match key:
            case 'w':
                self.msg.twist.linear.x = self.speed
            case 's':
                self.msg.twist.linear.x = -self.speed
            case 'd':
                self.msg.twist.linear.y = self.speed
            case 'a':
                self.msg.twist.linear.y = -self.speed
            case 'e':
                self.msg.twist.linear.z = self.speed
            case 'q':
                self.msg.twist.linear.z = -self.speed
            case 'C':
                self.msg.twist.angular.z = -self.yaw_rate
            case 'D':
                self.msg.twist.angular.z = self.yaw_rate

    def publish_command(self):
        # frame_id is deliberately left empty: MAVROS ignores it and uses the world frame, so
        # naming a body frame here would be a lie.
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    if not sys.stdin.isatty():
        print('teleop_velocity needs a terminal to read keys from', file=sys.stderr)
        return

    rclpy.init()
    node = None
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        node = TeleopVelocity()
        tty.setraw(fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as err:
        print(f'teleop_velocity: {err}', file=sys.stderr)
    finally:
        # Runs on every exit path, including Ctrl-C, so the shell is never left in raw mode.
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
