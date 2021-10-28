#!/usr/bin/env python
#coding=utf-8
# created by Wu GengQian

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

rospy.init_node("disparity_extender")
speed_param = rospy.get_param('~speed_param', 3.5)
P_param = rospy.get_param('~P_param', 0.0)

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

DISPARITY_DIF = 0.6
CAR_WIDTH = 0.16

# 里程计估计的当前位姿，仿真环境下很准确
pose_x = 0
pose_y = 0
pose_yaw = 0

def disparity_extender_callback(data):
    global pose_x
    global pose_y
    global pose_yaw
    global speed
    global P

    dis = []
    for angle in range(-90, 91):
        dis.append(get_range(data, angle))

    disparities = []
    for i in range(len(dis)):
        if i == len(dis) - 1:
            continue
        if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
            min_dis = min(dis[i], dis[i + 1])
            angle_range = math.ceil(
                math.degrees(math.atan(CAR_WIDTH / 2 / min_dis)))
            angle_range += 15
            side_range = range(int(i - angle_range + 1), i + 1) if dis[i + 1] == min_dis else range(i + 1, int(i + 1 + angle_range))
            disparities.append((min_dis, side_range))

    for min_dis, side_range in disparities:
        for i in side_range:
            if i >= 0 and i < len(dis):
                dis[i] = min(dis[i], min_dis)

    # 对开放地图特定位置的激光数据进行裁剪，以下适用于levine这张地图
    if (pose_x < -12 and pose_y > 7):
        for i in range(0, 90):
            dis[i] = 0
    elif (pose_x < -13.5 and pose_y < 1.5):
        for i in range(0, 120):
            dis[i] = 0
    elif (pose_x > 8 and pose_y < 1):
        for i in range(0, 90):
            dis[i] = 0

    max_index = np.argmax(dis)
    max_dis = dis[max_index]

    angle = max_index - 90 if abs(max_index - 90) > 15 else 0
    angle = angle * np.pi / 180
    # speed = 3.5
    speed = speed_param + P_param * abs(angle)

    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = angle
    drive_msg.drive.speed = speed
    drive_pub.publish(drive_msg)

def odom_callback(data):
    global pose_x
    global pose_y
    global pose_yaw
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    orientation = data.pose.pose.orientation
    (roll, pitch, pose_yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

if __name__ == '__main__': 
  try:
    scan_sub = rospy.Subscriber('/scan', LaserScan, disparity_extender_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
