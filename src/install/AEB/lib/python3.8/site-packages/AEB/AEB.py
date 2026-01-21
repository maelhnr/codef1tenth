#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class AEB(Node):

    def __init__(self):
        super().__init__('AEB')

        # Initialize variables
        self.speed = 0.0
        # Create ROS subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.brk_pub = self.create_publisher(AckermannDriveStamped, 'brake', 10)
        
    def odom_callback(self, odom_msg):
        # Update current speed
        self.speed = odom_msg.twist.twist.linear.x
        
    def scan_callback(self, scan_msg):
        # Calculate TTC
        ranges = scan_msg.ranges
        range_max = scan_msg.range_max
        range_min = scan_msg.range_min
        valid_ranges = [x for x in ranges if range_min <= x <= range_max]

        ttc = []
        for i in range(len(valid_ranges)):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            projection = abs(self.speed) * math.cos(angle)
            denominator = max(-projection, 0.0)
            if self.speed > 0:
                ttc.append(valid_ranges[i] / denominator)

        # Publish brake message and publish controller bool
        for time in ttc:
            print(ttc)
            if time < 10:
                ack_msg = AckermannDriveStamped()
                ack_msg.header.stamp = self.get_clock().now().to_msg()
                ack_msg.header.frame_id = 'AEB'
                ack_msg.drive.speed = 0.0
                self.brk_pub.publish(ack_msg)

def main():
    rclpy.init()
    aeb = AEB()
    rclpy.spin(aeb)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

