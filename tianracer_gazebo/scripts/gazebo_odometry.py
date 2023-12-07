#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3Stamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
import tf2_ros
import tf2_geometry_msgs

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/tianracer/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_pose.orientation.w = 1
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index('tianracer::tianracer/base_footprint')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
            
            # get a Vector3Stamped from the twist_in_gazebo
            twist_linear = Vector3Stamped()
            twist_linear.vector = self.last_received_twist.linear
            
            # get a TransformStamped from the last_received_pose
            twist_transform = TransformStamped()
            
            # last received pose is base_footprint in gazebo. twist is in gazebo frame.
            # we want to transform the twist into the base_footprint frame. requires a inverse
            q=tf.transformations.quaternion_inverse([self.last_received_pose.orientation.x, self.last_received_pose.orientation.y, self.last_received_pose.orientation.z, self.last_received_pose.orientation.w])
            
            twist_transform.transform.rotation.x = q[0]
            twist_transform.transform.rotation.y = q[1]
            twist_transform.transform.rotation.z = q[2]
            twist_transform.transform.rotation.w = q[3]
                
            # transform the Vector3Stamped into the child frame, base_link
            twist_linear = tf2_geometry_msgs.do_transform_vector3(twist_linear, twist_transform)
            
            ######### the result is not proper ###########
            #print(twist_linear)

            self.last_received_twist.linear = twist_linear.vector
        
        self.last_recieved_stamp = rospy.Time.now()
          
            
    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        odom = Odometry()
        odom.header.stamp = self.last_recieved_stamp
        odom.header.frame_id = '/tianracer/odom'
        odom.child_frame_id = '/tianracer/base_footprint'
        odom.pose.pose = self.last_received_pose
        # print("pose ", self.last_received_pose)
        odom.twist.twist = self.last_received_twist
        odom.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3]

        odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


        self.pub_odom.publish(odom)
        # print('gazebo_odom changing')
        tf = TransformStamped(
            header=Header(
                frame_id=odom.header.frame_id,
                stamp=odom.header.stamp
            ),
            child_frame_id=odom.child_frame_id,
            transform=Transform(
                translation=odom.pose.pose.position,
                rotation=odom.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()
