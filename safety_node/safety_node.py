#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribers        #self.get_logger().info(f'BRAKE! Minimum TTC: s')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        
        # Variables
        self.current_speed = 0.0
        self.latest_scan = None
        
        # Timer for periodic checks
        self.timer = self.create_timer(0.001, self.timer_callback)  # 10 Hz
        
        # Parameters
        self.ttc_threshold = 0.7  # seconds
        
        self.get_logger().info("Safety node initialized")

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is None:
            return
        
        scan = self.latest_scan
        vx = self.current_speed
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        range_min = scan.range_min
        range_max = scan.range_max
        
        min_ttc = float('inf')
        
        for i, r in enumerate(ranges):
            # Check for valid range measurement
            if not math.isfinite(r):
                continue
            if r < range_min or r > range_max:
                continue
            
            theta = angle_min + i * angle_increment
            projected_velocity = vx * math.cos(theta)
            
            if projected_velocity <= 0:
                continue  # No imminent collision in this direction
            
            ttc = r / projected_velocity
            if ttc < min_ttc:
                min_ttc = ttc
        
        if min_ttc < self.ttc_threshold:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            for i in range(1000):
                self.drive_pub.publish(drive_msg)
                self.get_logger().info(f'BRAKE! Minimum TTC: {min_ttc:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
