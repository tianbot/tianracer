#! /usr/bin/env python3
# Author: christoph.roesmann@tu-dortmund.dem， Kong Liangqian

import rclpy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def cmd_callback(data, wheelbase, ackermann_cmd_topic, pub):
    
    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
    msg = AckermannDrive()
    msg.steering_angle = float(steering)
    msg.speed = v
    
    pub.publish(msg)


def main():
    
    rclpy.init()
    node = rclpy.create_node('cmd_vel_to_ackermann_drive')

    # declare and read parameters (use default values when not provided)
    twist_cmd_topic = node.declare_parameter('twist_cmd_topic', '/cmd_vel').value
    ackermann_cmd_topic = node.declare_parameter('ackermann_cmd_topic', '/tianracer/ackermann_cmd').value
    wheelbase = node.declare_parameter('wheelbase', 0.255).value

    pub = node.create_publisher(AckermannDrive, ackermann_cmd_topic, 10)
    node.create_subscription(Twist, twist_cmd_topic, lambda x: cmd_callback(x, wheelbase, ackermann_cmd_topic, pub), 10)

    node.get_logger().info("Node 'cmd_vel_to_ackermann_drive' started. Listening to %s, publishing to %s. wheelbase: %f" % (twist_cmd_topic, ackermann_cmd_topic, wheelbase))

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__': 
    main()

