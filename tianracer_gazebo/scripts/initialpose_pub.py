#! /usr/bin/env python3 
# -*- coding: utf-8 -*-

# Description: publish initial pose to ros topic
# Author: sujit-168 su2054552689@gmail.com
# Date: 2024-03-22 17:40:53

import os
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

robot_name = os.getenv("TIANRACER_NAME", "tianracer")

class InitialPosePublisher():
    def __init__(self, rate):
        super(InitialPosePublisher, self).__init__()
        self.publisher = rospy.Publisher(f"{robot_name}/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.get_param()  # get params
        self.done = False
    
    def get_param(self):
        self.x_pos = rospy.get_param("~x_pos", 0.0)
        self.y_pos = rospy.get_param("~y_pos", 0.0)
        self.z_pos = rospy.get_param("~z_pos", 1.0)
        self.R_pos = rospy.get_param("~R_pos", 0.0)
        self.P_pos = rospy.get_param("~P_pos", 0.0)
        self.Y_pos = rospy.get_param("~Y_pos", 1.54)  # rad / degree 0' = degree * pi / 180

        self.print_params()

    def print_params(self):
        print("Initial pose: x={:.2f}, y={:.2f}, z={:.2f}, R={:.2f}, P={:.2f}, Y={:.2f}".format(self.x_pos, self.y_pos, self.z_pos, self.R_pos, self.P_pos, self.Y_pos))
    
    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if  i== 4:
                rospy.loginfo("Waiting for subscribers to connect to /initialpose...")
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Gave up waiting for subscribers")
        
    def run(self):
        self.wait_for_subscribers()  # Call wait_for_subscribers before publishing

        pose_msg = PoseWithCovarianceStamped()
        while not self.done:
            
            # 设置消息头
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # 设置 pose 消息
            pose_msg.pose.pose.position.x = self.x_pos
            pose_msg.pose.pose.position.y = self.y_pos
            pose_msg.pose.pose.position.z = self.z_pos
            
            # 将四元数转换为欧拉角
            quat = tf.transformations.quaternion_from_euler(self.R_pos, self.P_pos, self.Y_pos)
            pose_msg.pose.pose.orientation.x = quat[0]
            pose_msg.pose.pose.orientation.y = quat[1]
            pose_msg.pose.pose.orientation.z = quat[2]
            pose_msg.pose.pose.orientation.w = quat[3]

            # 设置协方差矩阵
            pose_msg.pose.covariance[0] = 0.1
            pose_msg.pose.covariance[7] = 0.1
            pose_msg.pose.covariance[14] = 0.1
            pose_msg.pose.covariance[21] = 0.1
            pose_msg.pose.covariance[28] = 0.1
            pose_msg.pose.covariance[35] = 0.1

            # 发布消息
            # rospy.loginfo("Publishing initialpose message: %s", pose_msg)
            self.publisher.publish(pose_msg)

            # Check for rospy.is_shutdown() to allow termination with Ctrl+C
            if rospy.is_shutdown():
                    break
    
    def run_timer(self, time):
        """Publishes the initial pose for a specified time duration."""
        self.wait_for_subscribers()

        pose_msg = PoseWithCovarianceStamped()
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < time:
            # 设置消息头
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # 设置 pose 消息
            pose_msg.pose.pose.position.x = self.x_pos
            pose_msg.pose.pose.position.y = self.y_pos
            pose_msg.pose.pose.position.z = self.z_pos
            
            # 将四元数转换为欧拉角
            quat = tf.transformations.quaternion_from_euler(self.R_pos, self.P_pos, self.Y_pos)
            pose_msg.pose.pose.orientation.x = quat[0]
            pose_msg.pose.pose.orientation.y = quat[1]
            pose_msg.pose.pose.orientation.z = quat[2]
            pose_msg.pose.pose.orientation.w = quat[3]

            # 设置协方差矩阵
            pose_msg.pose.covariance[0] = 0.1
            pose_msg.pose.covariance[7] = 0.1
            pose_msg.pose.covariance[14] = 0.1
            pose_msg.pose.covariance[21] = 0.1
            pose_msg.pose.covariance[28] = 0.1
            pose_msg.pose.covariance[35] = 0.1

            # rospy.loginfo("Publishing initialpose message: %s", pose_msg)
            self.publisher.publish(pose_msg)

        exit()

def test():
    try:
        rospy.init_node('initial_pose_publisher')
        # InitialPosePublisher(10).run()
        InitialPosePublisher(1).run_timer(0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
if __name__ == "__main__":
    test()