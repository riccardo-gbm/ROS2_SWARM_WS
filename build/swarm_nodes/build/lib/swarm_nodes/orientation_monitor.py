import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

NAMES = {'drone1': 'PAPOI', 'drone2': 'TRINEO', 'drone3': 'BISUU', 'drone4': 'LUCAS'}


def to_euler_deg(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class OrientationMonitor(Node):
    def __init__(self):
        super().__init__('orientation_monitor')
        self.rpy = {ns: None for ns in NAMES}
        self.hdg = {ns: None for ns in NAMES}
        for ns in NAMES:
            self.create_subscription(Imu, f'/{ns}/mavros/imu/data',
                                     lambda m, ns=ns: self.on_imu(ns, m), qos_profile_sensor_data)
            self.create_subscription(Float64, f'/{ns}/mavros/global_position/compass_hdg',
                                     lambda m, ns=ns: self.on_hdg(ns, m), qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_imu(self, ns, msg):
        self.rpy[ns] = to_euler_deg(msg.orientation)

    def on_hdg(self, ns, msg):
        self.hdg[ns] = msg.data

    def report(self):
        lines = ['']
        for ns, name in NAMES.items():
            if self.rpy[ns] is None:
                lines.append(f'{name:7s} no IMU data')
                continue
            roll, pitch, yaw_enu = self.rpy[ns]
            heading = (90.0 - yaw_enu) % 360.0  # ENU yaw (0 = east) -> compass (0 = north, clockwise)
            compass = f'{self.hdg[ns]:6.1f}' if self.hdg[ns] is not None else '   n/a'
            lines.append(f'{name:7s} roll {roll:6.1f}  pitch {pitch:6.1f}  '
                         f'heading(imu) {heading:6.1f}  heading(compass) {compass}')
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    rclpy.spin(OrientationMonitor())


if __name__ == '__main__':
    main()