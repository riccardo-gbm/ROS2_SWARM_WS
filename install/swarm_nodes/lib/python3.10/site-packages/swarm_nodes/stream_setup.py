"""Request MAVLink message rates from every drone, and re-request them after a reboot.

MAVLink ids: 30 ATTITUDE, 31 ATTITUDE_QUATERNION, 32 LOCAL_POSITION_NED, 33 GLOBAL_POSITION_INT
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval


class StreamSetup(Node):
    def __init__(self):
        super().__init__('stream_setup')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        self.message_ids = self.declare_parameter('message_ids', [30, 31, 32, 33]).value
        self.rate_hz = self.declare_parameter('rate_hz', 25.0).value
        reapply_period = self.declare_parameter('reapply_period', 30.0).value

        self.connected = {ns: False for ns in self.drone_ns}
        self.interval_clients = {}
        for ns in self.drone_ns:
            self.interval_clients[ns] = self.create_client(
                MessageInterval, f'/{ns}/mavros/set_message_interval')
            # best-effort + volatile subscribes to any publisher QoS
            self.create_subscription(State, f'/{ns}/mavros/state',
                                     lambda msg, ns=ns: self.on_state(ns, msg),
                                     qos_profile_sensor_data)
        if reapply_period > 0:
            self.create_timer(reapply_period, self.reapply_all)

    def on_state(self, ns, msg):
        if msg.connected and not self.connected[ns]:
            if self.request(ns):
                self.get_logger().info(f'{ns}: connected, message rates requested')
                self.connected[ns] = True
        elif not msg.connected and self.connected[ns]:
            self.get_logger().warn(f'{ns}: connection lost, will re-request on reconnect')
            self.connected[ns] = False

    def reapply_all(self):
        for ns in self.drone_ns:
            if self.connected[ns]:
                self.request(ns, quiet=True)

    def request(self, ns, quiet=False):
        client = self.interval_clients[ns]
        if not client.service_is_ready():
            return False  # MAVROS not up yet; retried on the next state message
        for msg_id in self.message_ids:
            req = MessageInterval.Request()
            req.message_id = int(msg_id)
            req.message_rate = float(self.rate_hz)
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, ns=ns, msg_id=msg_id: self.on_done(ns, msg_id, f, quiet))
        return True

    def on_done(self, ns, msg_id, future, quiet):
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(f'{ns}: rate request for message {msg_id} failed')
        elif not quiet:
            self.get_logger().info(f'{ns}: message {msg_id} at {self.rate_hz:.0f} Hz')


def main():
    rclpy.init()
    rclpy.spin(StreamSetup())


if __name__ == '__main__':
    main()