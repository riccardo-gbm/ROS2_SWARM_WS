#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped

class PublishPoseNode(Node):

    def __init__(self):
        super().__init__("fetch_pose")

        frecuency = 1 / 25

        self.msg = None
        self.subscribe_to_pose_ = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.read_pose, qos_profile_sensor_data)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "nav_msgs/pose", 20)

        self.timer = self.create_timer(frecuency, self.timer_callback)

    def read_pose(self, msg):
        self.msg = msg
        
    def timer_callback(self):
        if self.msg:
            self.pose_publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    publish_pose_node = PublishPoseNode()
    rclpy.spin(publish_pose_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()