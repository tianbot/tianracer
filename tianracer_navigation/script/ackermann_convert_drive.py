#!/usr/bin/env python3
# Author: tianbot
# Description:  Convert ackermann_msgs/AckermannDriveStamped to ackermann_msgs/AckermannDrive

import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist

class AckermannConverter:
    def __init__(self):
        rospy.init_node('ackermann_stamped_converter', anonymous=True) 
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', 'ackermann_cmd')
        ackermann_stamped_cmd_topic = rospy.get_param('~ackermann_stamped_cmd_topic', 'ackermann_cmd_stamped')

        # create publisher（deafult /ackermann_cmd）
        self.drive_pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
        
        # create subscrriber（default: ackermann_cmd_stamped）
        rospy.Subscriber(ackermann_stamped_cmd_topic, AckermannDriveStamped, self.convert_callback, queue_size=1)

    def convert_callback(self, msg):
        """callback func"""

        drive_msg = AckermannDrive()
        
        # copy the message value
        drive_msg.speed = msg.drive.speed
        drive_msg.steering_angle = msg.drive.steering_angle
        drive_msg.acceleration = msg.drive.acceleration
        drive_msg.jerk = msg.drive.jerk
        
        # post the converted message
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        converter = AckermannConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass