import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import pygame

class TeleopVelocityCommand(Node):

    def __init__(self):
        super().__init__("teleop_velocity")

        # Velocity at which inputs will be processed and velocity commands sent.
        self.INPUT_FRECUENCY = 1/50

        # Pygame constants and parameters, used for the GUI.
        self.WINDOW_WIDTH = 900
        self.WINDOW_HEIGHT = 700
        self.VERTICAL_WINDOW_WIDTH = 300
        self.WINDOW_WIDTH_CENTER = (self.WINDOW_WIDTH - self.VERTICAL_WINDOW_WIDTH) / 2
        self.WINDOW_HEIGHT_CENTER = self.WINDOW_HEIGHT / 2
        self.VERTICAL_WINDOW_WIDTH_CENTER = self.WINDOW_WIDTH - (self.VERTICAL_WINDOW_WIDTH / 2)
        self.METER_TO_PIXEL = 10
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.WHITE = (255, 255, 255)
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

        pygame.init()
        self.screen = pygame.display.set_mode((self.WINDOW_WIDTH, self.WINDOW_HEIGHT))

        pygame.display.set_caption("Controller")

        self.declare_parameter("drone_id", "")
        self.drone_id = self.get_parameter("drone_id").value
        
        if self.drone_id:
            self.drone_id = "/" + self.drone_id

        self.pose_subscriber = self.create_subscription(PoseStamped, f"{self.drone_id}/mavros/local_position/pose", self.pose_callback, qos_profile_sensor_data)

        self.msg = TwistStamped()     

        self.publisher_vel_command = self.create_publisher(TwistStamped, f"{self.drone_id}/mavros/setpoint_velocity/cmd_vel", qos_profile_system_default)

        self.timer = self.create_timer(self.INPUT_FRECUENCY, self.user_input_to_command)

    def user_input_to_command(self):

        self.screen.fill((0, 0, 0))
        pygame.draw.rect(self.screen, self.WHITE, (self.WINDOW_WIDTH-self.VERTICAL_WINDOW_WIDTH, 0, 1, self.WINDOW_HEIGHT))
        pygame.draw.rect(self.screen, self.GREEN, (self.WINDOW_WIDTH_CENTER, self.WINDOW_HEIGHT_CENTER, 10, 10))
        pygame.draw.rect(self.screen, self.RED, (self.current_x, self.current_y, 10, 10))
        pygame.draw.rect(self.screen, self.RED, (self.VERTICAL_WINDOW_WIDTH_CENTER, self.current_z, 10, 10))

        pygame.display.flip()

        pressed_keys = pygame.key.get_pressed()

        x = 0.0
        y = 0.0
        z = 0.0
        r = 0.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        if pressed_keys[pygame.K_w]:
            x += 1
        if pressed_keys[pygame.K_s]:
            x -= 1
        if pressed_keys[pygame.K_d]:
          y += 1
        if pressed_keys[pygame.K_a]:
            y -= 1
        if pressed_keys[pygame.K_e]:
            z += 1
        if pressed_keys[pygame.K_q]:
            z -= 1
        if pressed_keys[pygame.K_RIGHT]:
            r -= 1
        if pressed_keys[pygame.K_LEFT]:
            r += 1

        self.msg.twist.linear.x = x
        self.msg.twist.linear.y = y
        self.msg.twist.linear.z = z
        self.msg.twist.angular.z = r
        
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "base_link"

        self.publisher_vel_command.publish(self.msg)

    def pose_callback(self, msg):
    
        self.current_y = int(-(msg.pose.position.x * self.METER_TO_PIXEL) + self.WINDOW_HEIGHT_CENTER)
        self.current_x = int((msg.pose.position.y * self.METER_TO_PIXEL) + self.WINDOW_WIDTH_CENTER)
        self.current_z = int(-(msg.pose.position.z * self.METER_TO_PIXEL) + self.WINDOW_HEIGHT-5)

def main(args=None):

    try:
        rclpy.init(args=args)
        publish_velocity_command = TeleopVelocityCommand()  
        rclpy.spin(publish_velocity_command)
      
    except Exception as error:
        publish_velocity_command.get_logger().error(f"Exception in TeleopVelocityCommand: {error}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()