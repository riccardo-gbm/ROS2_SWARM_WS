import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 



class OdomNode(Node):
    def __init__(self):
        super().__init__('nodo_odometria')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Suscripción al tópico de odometría de MAVROS
        self.sub_odom = self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_callback, qos_profile)
        
        # Publicador a mi propio tópico
        self.pub_odom = self.create_publisher(Odometry, '/mi_dron/odometria_republicada', qos_profile)
        
        self.get_logger().info('Nodo de odometria iniciado correctamente.')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.get_logger().info(f'Posicion actual -> X: {pos.x} | Y: {pos.y} | Z: {pos.z}')
        
        # Republicar mensaje
        self.pub_odom.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    node = OdomNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
