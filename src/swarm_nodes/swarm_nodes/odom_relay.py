"""Republish each drone's MAVROS odometry on the topic the formation controller subscribes to.

Subscribes /droneN/mavros/local_position/odom and publishes /droneN/odom at a fixed rate.

PASSTHROUGH ONLY. Each drone's pose is still expressed in its own EKF origin, set wherever that
vehicle first got a good GPS fix, so the four frames are not comparable yet. The shared swarm_origin
shift is a separate change; until it lands, frame_id is left exactly as MAVROS set it rather than
claiming to be swarm_origin.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        rate_hz = self.declare_parameter('rate_hz', 25.0).value

        # Seeded up front so the timer never iterates a dict the subscriptions are still growing.
        self.latest = {ns: None for ns in self.drone_ns}
        self.seen = {ns: False for ns in self.drone_ns}

        self.publishers_by_ns = {}
        for ns in self.drone_ns:
            self.publishers_by_ns[ns] = self.create_publisher(
                Odometry, f'/{ns}/odom', qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/mavros/local_position/odom',
                                     lambda msg, ns=ns: self.on_odom(ns, msg),
                                     qos_profile_sensor_data)

        self.create_timer(1.0 / rate_hz, self.publish_latest)
        self.get_logger().info(
            f'relaying {len(self.drone_ns)} drones to /<ns>/odom at {rate_hz:.0f} Hz')
        self.get_logger().warn(
            'passthrough mode: every pose is still in its own drone\'s EKF origin, so the four '
            'frames are NOT comparable. Shared swarm_origin shift is not implemented yet.')

    def on_odom(self, ns, msg):
        if not self.seen[ns]:
            self.get_logger().info(f'{ns}: first odometry message received')
            self.seen[ns] = True
        self.latest[ns] = msg

    def publish_latest(self):
        for ns in self.drone_ns:
            msg = self.latest[ns]
            if msg is None:
                continue  # nothing new since the last tick; don't republish a stale pose
            msg.child_frame_id = f'{ns}/base_link'
            self.publishers_by_ns[ns].publish(msg)
            self.latest[ns] = None


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())


if __name__ == '__main__':
    main()
