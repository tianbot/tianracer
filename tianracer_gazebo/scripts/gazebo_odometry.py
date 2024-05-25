#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
import numpy as np
import math
import tf2_ros

robot_name = rospy.get_param('robot_name', default="tianracer")
wheel_radius = 0.032  # Wheel radius in meters
wheel_base = 0.265     # Distance between left and right wheels

class OdometryNode:
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    def __init__(self):
        self.pose = Pose()
        self.pose.orientation.w = 1  # initial quaternion needs to be normalized
        self.velocity = 0  # Initialize linear velocity
        self.angular_velocity = 0  # Initialize angular velocity
        self.last_time = rospy.Time.now()
        self.last_left_wheel_position = 0
        self.last_right_wheel_position = 0
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/tianracer/joint_states', JointState, self.joint_state_callback)
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)  # 20 Hz

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        rospy.loginfo("dt: %f", dt)
        
        # The state of each joint (revolute or prismatic) is defined by:
        #  * the position of the joint (rad or m),
        #  * the velocity of the joint (rad/s or m/s) and 
        #  * the effort that is applied in the joint (Nm or N).

        left_wheel_idx = msg.name.index('left_rear_wheel_joint')
        right_wheel_idx = msg.name.index('right_rear_wheel_joint')
        
        left_wheel_position = msg.position[left_wheel_idx]
        right_wheel_position = msg.position[right_wheel_idx]

        # Calculate the traveled distance since last update
        delta_left_wheel = (left_wheel_position - self.last_left_wheel_position) * wheel_radius
        delta_right_wheel = (right_wheel_position - self.last_right_wheel_position) * wheel_radius

        # Calculate average forward velocity and angular velocity
        # self.velocity = (delta_left_wheel + delta_right_wheel) / 2 / dt
        # self.angular_velocity = (delta_right_wheel - delta_left_wheel) / wheel_base / dt
        self.velocity = (msg.velocity[left_wheel_idx] * wheel_radius + msg.velocity[right_wheel_idx] * wheel_radius) / 2
        self.angular_velocity = (msg.velocity[right_wheel_idx] * wheel_radius - msg.velocity[left_wheel_idx] * wheel_radius) / wheel_base
        
        # Update the pose
        self.pose.position.x += self.velocity * math.cos(self.pose.orientation.z) * dt
        self.pose.position.y += self.velocity * math.sin(self.pose.orientation.z) * dt
        self.pose.orientation.z += self.angular_velocity * dt

        # Update last positions and time
        self.last_left_wheel_position = left_wheel_position
        self.last_right_wheel_position = right_wheel_position
        self.last_time = current_time

    def timer_callback(self, event):
        odom = Odometry()
        odom.header.stamp = self.last_time
        odom.header.frame_id = f'{robot_name}/odom'
        odom.child_frame_id = f'{robot_name}/base_footprint'
        odom.pose.pose = self.pose
        odom.twist.twist.linear.x = self.velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.pub_odom.publish(odom)

        tf = TransformStamped(
            header=odom.header,
            child_frame_id=odom.child_frame_id,
            transform=Transform(
                translation=odom.pose.pose.position,
                rotation=odom.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
