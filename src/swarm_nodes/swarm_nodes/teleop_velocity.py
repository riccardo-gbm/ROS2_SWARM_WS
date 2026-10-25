#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistStamped
import sys
import select
import time
import tty
import termios

class TeleopVelocityCommand(Node):

    def __init__(self):
        super().__init__("teleop_velocity")

        self.INPUT_FRECUENCY = 1 / 1000
        self.FD = sys.stdin.fileno()
        self.OLD_SETTINGS = termios.tcgetattr(self.FD)
        self.KEY_TIMEOUT = 0.005

        self.declare_parameter("drone_id", "")
        self.drone_id = self.get_parameter("drone_id").value

        tty.setraw(self.FD)

        self.key_press = time.monotonic()
        self.msg = TwistStamped()     

        self.publisher_vel_command = self.create_publisher(TwistStamped, f"{self.drone_id}/mavros/setpoint_velocity/cmd_vel", qos_profile_system_default)

        self.timer = self.create_timer(self.INPUT_FRECUENCY, self.user_input_to_command)
        self.publish_vel_command_timer = self.create_timer(self.INPUT_FRECUENCY, self.publish_vel_command)

    def user_input_to_command(self):

        readable, _, _ = select.select([sys.stdin], [], [], 0)

        if readable:
            key = sys.stdin.read(1)
            if key == "\x1b":
                key += sys.stdin.read(2)
                key = key[-1]
            if key == '\x03':
                termios.tcsetattr(
                    self.FD, termios.TCSADRAIN, self.OLD_SETTINGS)
                raise KeyboardInterrupt

            self.key_press = time.monotonic()

            self.msg.twist.linear.x = 0.0
            self.msg.twist.linear.y = 0.0
            self.msg.twist.linear.z = 0.0
            self.msg.twist.angular.z = 0.0 

            match key:
                case "w":
                    self.msg.twist.linear.x = 1.0
                case "s":
                    self.msg.twist.linear.x = -1.0
                case "d":
                    self.msg.twist.linear.y = 1.0
                case "a":
                    self.msg.twist.linear.y = -1.0
                case "e":
                    self.msg.twist.linear.z = 1.0
                case "q":
                    self.msg.twist.linear.z = -1.0
                case "C":
                    self.msg.twist.angular.z = 1.0
                case "D":
                    self.msg.twist.angular.z = -1.0

        elif time.monotonic() - self.key_press > self.KEY_TIMEOUT:
            self.msg.twist.linear.x = 0.0
            self.msg.twist.linear.y = 0.0
            self.msg.twist.linear.z = 0.0
            self.msg.twist.angular.z = 0.0 
        
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "base_link"


    def publish_vel_command(self):
        self.publisher_vel_command.publish(self.msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        publish_velocity_command = TeleopVelocityCommand()
        rclpy.spin(publish_velocity_command)
        rclpy.shutdown()
    except Exception:
        termios.tcsetattr(
            sys.stdin.fileno(), termios.TCSADRAIN, publish_velocity_command.OLD_SETTINGS)


if __name__ == '__main__':
    main()