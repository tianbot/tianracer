#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class AckermannConverter(Node):
    def __init__(self):
        super().__init__('ackermann_stamped_converter')
        
        # Declare parameters
        self.declare_parameter('ackermann_cmd_topic', 'ackermann_cmd')
        self.declare_parameter('ackermann_stamped_cmd_topic', 'ackermann_cmd_stamped')
        
        # Get parameters
        ackermann_cmd_topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value
        ackermann_stamped_cmd_topic = self.get_parameter('ackermann_stamped_cmd_topic').get_parameter_value().string_value
        
        # Latest drive message buffer
        self.latest_drive_msg = None
        
        # Publisher
        self.drive_pub = self.create_publisher(AckermannDrive, ackermann_cmd_topic, 10)
        
        # Subscriber
        self.sub = self.create_subscription(AckermannDriveStamped, ackermann_stamped_cmd_topic, self.convert_callback, 10)
        
        # Timer (50Hz)
        self.timer = self.create_timer(1.0/50.0, self.timer_callback)
        
        self.get_logger().info(f"Node started at 50Hz. Subscribing to {ackermann_stamped_cmd_topic}, publishing to {ackermann_cmd_topic}")

    def convert_callback(self, msg):
        """Unpack the Stamped message and store it in the buffer"""
        if self.latest_drive_msg is None:
            self.latest_drive_msg = AckermannDrive()
            
        self.latest_drive_msg.speed = msg.drive.speed
        self.latest_drive_msg.steering_angle = msg.drive.steering_angle
        self.latest_drive_msg.acceleration = msg.drive.acceleration
        self.latest_drive_msg.steering_angle_velocity = msg.drive.steering_angle_velocity
        self.latest_drive_msg.jerk = msg.drive.jerk

    def timer_callback(self):
        """Continuously publish the latest cached data at 50Hz"""
        if self.latest_drive_msg is not None:
            self.drive_pub.publish(self.latest_drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
