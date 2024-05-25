#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist, Quaternion, Transform, TransformStamped
import numpy as np
import math
import tf2_ros
from tf.transformations import quaternion_from_euler, quaternion_multiply

robot_name = rospy.get_param('robot_name', default="tianracer")
wheel_radius = 0.032  # Wheel radius in meters
wheel_base = 0.265     # Distance between left and right wheels
wheel_width = 0.18

class OdometryNode:
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)

    def __init__(self):
        self.pose = Pose()
        self.pose.orientation.w = 1  # initial quaternion needs to be normalized
        self.theta = 0  # Initialize orientation
        self.velocity = 0  # Initialize linear velocity
        self.angular_velocity = 0  # Initialize angular velocity
        self.last_time = rospy.Time(0)  # Initialize time
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/tianracer/joint_states', JointState, self.joint_state_callback)
        rospy.Timer(rospy.Duration(0.02), self.timer_callback)  # 50 Hz

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Check if this is the first callback
        if self.last_time == rospy.Time(0):  # Initial condition check
            # Initialize last_time and last positions for wheels
            self.last_time = current_time
            self.left_wheel_idx = msg.name.index('left_rear_wheel_joint')
            self.right_wheel_idx = msg.name.index('right_rear_wheel_joint')
            return  # Skip the rest of the processing for the first callback
        
        # The state of each joint (revolute or prismatic) is defined by:
        #  * the position of the joint (rad or m),
        #  * the velocity of the joint (rad/s or m/s) and 
        #  * the effort that is applied in the joint (Nm or N).
        
        # Calculate average forward velocity and angular velocity
        self.velocity = (msg.velocity[self.left_wheel_idx] * wheel_radius + msg.velocity[self.right_wheel_idx] * wheel_radius) / 2
        self.angular_velocity = (msg.velocity[self.right_wheel_idx] * wheel_radius - msg.velocity[self.left_wheel_idx] * wheel_radius) / wheel_width
        
        # Update the pose
        delta_theta = self.angular_velocity * dt
        self.theta += delta_theta
        delta_x = self.velocity * math.cos(self.theta) * dt
        delta_y = self.velocity * math.sin(self.theta) * dt
        self.pose.position.x += delta_x
        self.pose.position.y += delta_y
        q = quaternion_from_euler(0, 0, self.theta)
        self.pose.orientation = Quaternion(*q)
         
        # Update time
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
