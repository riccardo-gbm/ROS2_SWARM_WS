import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

NAMES = {'drone1': 'PAPOI', 'drone2': 'TRINEO', 'drone3': 'BISUU', 'drone4': 'LUCAS'}


class LinkMonitor(Node):
    def __init__(self):
        super().__init__('link_monitor')
        self.stats = {ns: {'last': None, 'count': 0, 'max_gap': 0.0} for ns in NAMES}
        for ns in NAMES:
            self.create_subscription(Imu, f'/{ns}/mavros/imu/data',
                                     lambda msg, ns=ns: self.on_msg(ns),
                                     qos_profile_sensor_data)
        self.create_timer(5.0, self.report)

    def on_msg(self, ns):
        now = time.monotonic()
        s = self.stats[ns]
        if s['last'] is not None:
            s['max_gap'] = max(s['max_gap'], now - s['last'])
        s['last'] = now
        s['count'] += 1

    def report(self):
        parts = []
        for ns, s in self.stats.items():
            parts.append(f"{NAMES[ns]}: {s['count'] / 5.0:5.1f} Hz, max gap {s['max_gap'] * 1000:4.0f} ms")
            s['count'], s['max_gap'] = 0, 0.0
        self.get_logger().info(' | '.join(parts))


def main():
    rclpy.init()
    rclpy.spin(LinkMonitor())


if __name__ == '__main__':
    main()