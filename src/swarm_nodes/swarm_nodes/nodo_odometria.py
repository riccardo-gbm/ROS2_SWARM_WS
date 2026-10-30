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

        # Diccionario para almacenar el último mensaje recibido de cada dron
        self.latest_odom = {1: None, 2: None, 3: None, 4: None}
        
        # Suscripciones
        self.sub_odom1 = self.create_subscription(Odometry, '/drone1/mavros/local_position/odom', lambda msg: self.odom_callback(msg, 1), qos_profile)
        self.sub_odom2 = self.create_subscription(Odometry, '/drone2/mavros/local_position/odom', lambda msg: self.odom_callback(msg, 2), qos_profile)
        self.sub_odom3 = self.create_subscription(Odometry, '/drone3/mavros/local_position/odom', lambda msg: self.odom_callback(msg, 3), qos_profile)
        self.sub_odom4 = self.create_subscription(Odometry, '/drone4/mavros/local_position/odom', lambda msg: self.odom_callback(msg, 4), qos_profile)

        # Publicadores
        self.pub_odom1 = self.create_publisher(Odometry, '/drone1/odom', qos_profile)
        self.pub_odom2 = self.create_publisher(Odometry, '/drone2/odom', qos_profile)
        self.pub_odom3 = self.create_publisher(Odometry, '/drone3/odom', qos_profile)
        self.pub_odom4 = self.create_publisher(Odometry, '/drone4/odom', qos_profile)

        # Timer a 20 Hz
        rate_hz = 20.0
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.publish_timer_callback)

        self.get_logger().info('Nodo de odometria iniciado y configurado a 20 Hz.')

    def odom_callback(self, msg: Odometry, drone_id: int):
        # Guardar únicamente el último mensaje recibido en memoria
        self.latest_odom[drone_id] = msg

    def publish_timer_callback(self):
        # Este método se ejecuta 20 veces por segundo de forma constante
        for drone_id in range(1, 5):
            msg = self.latest_odom[drone_id]
            
            # Si no ha llegado información aún de este dron, saltar
            if msg is None:
                continue

            # Asignar frames
            msg.header.frame_id = 'swarm_origin'
            msg.child_frame_id = f'drone{drone_id}/base_link'

            # Publicar según el ID del dron
            if drone_id == 1:
                self.pub_odom1.publish(msg)
            elif drone_id == 2:
                self.pub_odom2.publish(msg)
            elif drone_id == 3:
                self.pub_odom3.publish(msg)
            elif drone_id == 4:
                self.pub_odom4.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()