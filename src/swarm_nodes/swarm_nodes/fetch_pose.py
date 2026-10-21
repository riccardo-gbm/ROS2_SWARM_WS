#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from functools import partial

class PublishPoseNode(Node):

    def __init__(self):
        super().__init__("fetch_pose")

        self.FRECUENCY = 1 / 25
        self.AMOUNT = 4
        self.count = 0
        self.messages = {}
        self.subscribers = []
        self.publisher = []

        for i in range(1, self.AMOUNT+1):
            self.subscribe_to_pose_ = self.create_subscription(Odometry, f"/drone{i}/mavros/local_position/odom", partial(self.read_pose, id_arg = i), qos_profile_sensor_data)
            self.publisher_ = self.create_publisher(Odometry, f"/drone{i}/nav_msgs/Odometry", qos_profile_sensor_data)
            self.subscribers.append(self.subscribe_to_pose_)
            self.publisher.append(self.publisher_)

        self.timer = self.create_timer(self.FRECUENCY, self.timer_callback)

    def read_pose(self, msg, id_arg):
        self.messages[id_arg] = msg

    def timer_callback(self):
        for keys in self.messages.keys():
            if self.messages[keys]:
                self.publisher[keys-1].publish(self.messages[keys])
                self.messages[keys] = None
    
def main(args=None):
    rclpy.init(args=args)
    publish_pose_node = PublishPoseNode()
    rclpy.spin(publish_pose_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()