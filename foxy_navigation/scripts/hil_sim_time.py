#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
import numpy as np
import time

from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory
        
class OdometryTime(Node):
    def __init__(self):
        super().__init__('odometry_time')
        
        self.vehicle_odometry = Odometry()

        self.encoder_sub = self.create_subscription(Odometry, '/odom_enc', self.encoder_cb, 1)
        self.encoder_pub = self.create_publisher(Odometry, '/odom_enc_sim', 1)

        self.step_timer = self.create_timer(0.05, self.step)
        
    def step(self):
        self.vehicle_odometry.header.stamp = self.get_clock().now().to_msg()
        self.encoder_pub.publish(self.vehicle_odometry)

    def encoder_cb(self, msg):
        self.vehicle_odometry.twist.twist.linear.x = round(msg.twist.twist.linear.x, 2)
        self.vehicle_odometry.twist.twist.linear.y = round(msg.twist.twist.linear.y, 2)
        self.vehicle_odometry.twist.twist.angular.z = round(msg.twist.twist.angular.z, 2)
        
def main(args=None):
    rclpy.init(args=args)
    my_node = OdometryTime() 
    rclpy.spin(my_node) 
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()