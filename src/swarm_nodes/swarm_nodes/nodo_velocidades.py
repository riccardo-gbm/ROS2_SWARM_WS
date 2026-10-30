import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CmdVelBridgeNode(Node):

    def __init__(self):
        super().__init__('nodo_velocidades')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publicadores hacia MAVROS para los 4 drones
        self.pub_cmd1 = self.create_publisher(Twist, '/drone1/mavros/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.pub_cmd2 = self.create_publisher(Twist, '/drone2/mavros/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.pub_cmd3 = self.create_publisher(Twist, '/drone3/mavros/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.pub_cmd4 = self.create_publisher(Twist, '/drone4/mavros/setpoint_velocity/cmd_vel_unstamped', qos_profile)

        self.get_logger().info('Nodo nodo_velocidades iniciado correctamente.')

    def enviar_velocidad_por_tiempo(self, drone_id: int, vx: float, vy: float, vz: float, wz: float, duracion_segundos: float):
        """Envia una velocidad a un dron durante cierto tiempo y luego lo frena."""
        
        # Crear mensaje con la velocidad deseada
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(wz)

        # Seleccionar el publicador
        publisher = self.obtener_publicador(drone_id)
        if publisher is None:
            self.get_logger().error(f'Drone ID {drone_id} no valido.')
            return

        self.get_logger().info(f'Enviando velocidad a Drone {drone_id} durante {duracion_segundos}s -> VX: {vx} | VY: {vy}')

        # Enviar el comando repetidamente a 10 Hz durante el tiempo especificado
        rate_hz = 10.0
        periodo = 1.0 / rate_hz
        tiempo_inicio = time.time()

        while (time.time() - tiempo_inicio) < duracion_segundos:
            publisher.publish(msg)
            time.sleep(periodo)

        # FRENADO DE SEGURIDAD: Enviar velocidad 0 al terminar la duracion
        msg_stop = Twist() # Todo en 0 por defecto
        publisher.publish(msg_stop)
        self.get_logger().info(f'Tiempo finalizado. Drone {drone_id} detenido.')

    def obtener_publicador(self, drone_id: int):
        if drone_id == 1: return self.pub_cmd1
        elif drone_id == 2: return self.pub_cmd2
        elif drone_id == 3: return self.pub_cmd3
        elif drone_id == 4: return self.pub_cmd4
        return None


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()

    rclpy.shutdown()

if __name__ == '__main__':
    main()