#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def wall_following_callback(data):
    """
    Implements simple wall following at
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
    """
    # the angle between the two laser rays
    THETA = np.pi / 180 * 60
    # target distance from wall
    TARGET_DIS = 1
    # the distance to project the car forward
    LOOK_AHEAD_DIS = 3
    P = 0.5

    # naming convention according to above pdf
    b = get_range(data, -90)
    a = get_range(data, -90 + np.rad2deg(THETA))
    # print(f"a{a:1.1f} b{b:1.1f}")
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)
    projected_dis = AB + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    steering_angle = P * error

    front_dis = get_range(data, 0)
    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    speed = 3 
    angle_filter = steering_angle
    
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle 
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass