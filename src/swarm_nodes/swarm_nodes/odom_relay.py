"""Republish each drone's MAVROS odometry on the topic the formation controller subscribes to.

Subscribes /droneN/mavros/local_position/odom and publishes /droneN/odom.

Every message is forwarded as it arrives, so the output rate equals the input rate. The autopilot
is what sets that rate -- stream_setup requests LOCAL_POSITION_NED at 25 Hz -- and this node must
not resample it. An earlier version latched the newest message and published it on a 25 Hz timer,
which aliased against a 25 Hz input and silently lost about a quarter of the messages.

max_rate_hz is an optional ceiling for the case where MAVLink is deliberately streaming faster than
/droneN/odom needs. It defaults to 0.0 (no limit) and only ever drops messages that genuinely
exceed the cap. It is a bound, not a target: since whole messages are dropped, the result lands at
or just below the cap depending on the input phase (a 10 Hz cap on a 50 Hz input gives ~9 Hz).

PASSTHROUGH ONLY. Each drone's pose is still expressed in its own EKF origin, set wherever that
vehicle first got a good GPS fix, so the four frames are not comparable yet. The shared swarm_origin
shift is a separate change; until it lands, frame_id is left exactly as MAVROS set it rather than
claiming to be swarm_origin.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        max_rate_hz = self.declare_parameter('max_rate_hz', 0.0).value
        self.min_period = 1.0 / max_rate_hz if max_rate_hz > 0.0 else 0.0

        self.seen = {ns: False for ns in self.drone_ns}
        self.last_pub = {ns: 0.0 for ns in self.drone_ns}

        self.publishers_by_ns = {}
        for ns in self.drone_ns:
            self.publishers_by_ns[ns] = self.create_publisher(
                Odometry, f'/{ns}/odom', qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/mavros/local_position/odom',
                                     lambda msg, ns=ns: self.on_odom(ns, msg),
                                     qos_profile_sensor_data)

        limit = f'{max_rate_hz:.0f} Hz ceiling' if self.min_period else 'no rate limit'
        self.get_logger().info(
            f'relaying {len(self.drone_ns)} drones to /<ns>/odom ({limit})')
        self.get_logger().warn(
            'passthrough mode: every pose is still in its own drone\'s EKF origin, so the four '
            'frames are NOT comparable. Shared swarm_origin shift is not implemented yet.')

    def on_odom(self, ns, msg):
        if not self.seen[ns]:
            self.get_logger().info(f'{ns}: first odometry message received')
            self.seen[ns] = True

        if self.min_period:
            now = time.monotonic()
            if now - self.last_pub[ns] < self.min_period:
                return
            self.last_pub[ns] = now

        msg.child_frame_id = f'{ns}/base_link'
        self.publishers_by_ns[ns].publish(msg)


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())


if __name__ == '__main__':
    main()
