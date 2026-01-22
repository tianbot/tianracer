#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')
        
        # Declare parameters
        self.declare_parameter('twist_cmd_topic', 'cmd_vel')
        self.declare_parameter('ackermann_cmd_topic', 'ackermann_cmd')
        self.declare_parameter('wheelbase', 1.0)
        
        # Get parameters
        twist_cmd_topic = self.get_parameter('twist_cmd_topic').get_parameter_value().string_value
        ackermann_cmd_topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        
        # Internal state
        self.current_v = 0.0
        self.current_steering = 0.0
        self.has_received_data = False
        
        # Publisher and Subscriber
        self.pub = self.create_publisher(AckermannDrive, ackermann_cmd_topic, 10)
        self.sub = self.create_subscription(Twist, twist_cmd_topic, self.cmd_callback, 10)
        
        # Timer (50Hz)
        self.timer = self.create_timer(1.0/50.0, self.timer_callback)
        
        self.get_logger().info(f"Node started at 50Hz. Wheelbase: {self.wheelbase}")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0:
            return 0.0
        
        if v == 0:
            v_eff = 0.1
        else:
            v_eff = v
            
        steering = math.atan(wheelbase * omega / v_eff)
        return steering

    def cmd_callback(self, msg):
        self.has_received_data = True
        v = msg.linear.x
        omega = msg.angular.z
        
        # Calculate steering angle
        self.current_steering = self.convert_trans_rot_vel_to_steering_angle(v, omega, self.wheelbase)
        
        # Velocity threshold clamping logic
        threshold = 0.5
        if 0 < v < threshold:
            v = threshold
        elif -threshold < v < 0:
            v = -threshold
        self.current_v = v

    def timer_callback(self):
        if self.has_received_data:
            msg = AckermannDrive()
            msg.speed = self.current_v
            msg.steering_angle = self.current_steering
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
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
