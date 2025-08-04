#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
import numpy as np
import time

from sensor_msgs.msg import Imu, LaserScan

from ament_index_python.packages import get_package_share_directory
        
class SimToHilTime(Node):
    def __init__(self):
        super().__init__('hil_time')
        
        self.imu_value = Imu()
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_cb, 1)
        self.imu_pub = self.create_publisher(Imu, '/imu', 1)

        self.laser_front_value = LaserScan()
        self.laser_front_sub = self.create_subscription(LaserScan, '/laser_horizontal_front_gazebo', self.laser_front_cb, 1)
        self.laser_front_pub = self.create_publisher(LaserScan, '/laser_horizontal_front_link', 1)

        self.laser_rear_value = LaserScan()
        self.laser_rear_sub = self.create_subscription(LaserScan, '/laser_horizontal_rear_gazebo', self.laser_rear_cb, 1)
        self.laser_rear_pub = self.create_publisher(LaserScan, '/laser_horizontal_rear_link', 1)

        self.laser_top_value = LaserScan()
        self.laser_top_sub = self.create_subscription(LaserScan, '/laser_horizontal_top_gazebo', self.laser_top_cb, 1)
        self.laser_top_pub = self.create_publisher(LaserScan, '/laser_horizontal_top_link', 1)

    def imu_cb(self, msg):
        self.imu_value = msg
        self.imu_value.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(self.imu_value)

    def laser_front_cb(self, msg):
        self.laser_front_value = msg
        self.laser_front_value.header.stamp = self.get_clock().now().to_msg()
        self.laser_front_pub.publish(self.laser_front_value)
    
    def laser_rear_cb(self, msg):
        self.laser_rear_value = msg
        self.laser_rear_value.header.stamp = self.get_clock().now().to_msg()
        self.laser_rear_pub.publish(self.laser_rear_value)

    def laser_top_cb(self, msg):
        self.laser_top_value = msg
        self.laser_top_value.header.stamp = self.get_clock().now().to_msg()
        self.laser_top_pub.publish(self.laser_top_value)

def main(args=None):
    rclpy.init(args=args)
    my_node = SimToHilTime() 
    rclpy.spin(my_node) 
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()