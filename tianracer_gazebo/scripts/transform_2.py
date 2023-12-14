#!/usr/bin/env python
import rospy
import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

import time 
import threading
<<<<<<< HEAD
pub = rospy.Publisher("/tianracer_01/ackermann_cmd_stamped", AckermannDriveStamped,queue_size=1)
=======
robot_name = rospy.get_param('robot_name', default="tianracer")
pub = rospy.Publisher("ackermann_cmd_stamped", AckermannDriveStamped,queue_size=1)
>>>>>>> origin/feature_one_sim

def thread_job():
    rospy.spin()

def callback(data):
    speed = data.linear.x 
    turn = data.angular.z

<<<<<<< HEAD
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";

    msg.drive.speed = speed;
    msg.drive.acceleration = 1;
    msg.drive.jerk = 1;
=======
    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = f"/{robot_name}/base_footprint"

    msg.drive.speed = speed
    msg.drive.acceleration = 1
    msg.drive.jerk = 1
>>>>>>> origin/feature_one_sim
    msg.drive.steering_angle = turn
    msg.drive.steering_angle_velocity = 1

    pub.publish(msg)

def SubscribeAndPublish():
    rospy.init_node('nav_sim', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    #rospy.Subscriber('cmd_vel', Twist, callback,queue_size=1,buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    try:
        SubscribeAndPublish()
    except rospy.ROSInterruptException:
        pass


########################
