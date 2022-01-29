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
speed_param = rospy.get_param('~speed_param', 3.5) # 小车行驶速度
P_param = rospy.get_param('~P_param', 0.0) # 根据转角小车转弯减速的比例 

FILTER_VALUE = 10.0
# 获取激光雷达测量的距离
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

DISPARITY_DIF = 0.6 # 判断是否触碰障碍物的阈值
CAR_WIDTH = 0.16 # 小车宽度，从simulator的params.yaml获取

# 激光数据/scan的回调函数
def disparity_extender_callback(data):

    dis = []
    # 获取前向180°的测量距离
    for angle in range(-90, 91):
        dis.append(get_range(data, angle))

    disparities = []
    for i in range(len(dis)):
        if i == len(dis) - 1:
            continue
        # 超过阈值则认为可能发生碰撞
        if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
            min_dis = min(dis[i], dis[i + 1]) # 计算可能碰撞最小距离
            angle_range = math.ceil(
                math.degrees(math.atan(CAR_WIDTH / 2 / min_dis))) # 根据小车宽度计算不会碰撞的最小角度
            angle_range += 15 # 添加容差
            # 需要裁剪的激光数据范围
            side_range = range(int(i - angle_range + 1), i + 1) if dis[i + 1] == min_dis else range(i + 1, int(i + 1 + angle_range))
            disparities.append((min_dis, side_range))

    # 裁剪激光数据
    for min_dis, side_range in disparities:
        for i in side_range:
            if i >= 0 and i < len(dis):
                dis[i] = min(dis[i], min_dis)

    max_index = np.argmax(dis) # 得到最可行距离的角度
    max_dis = dis[max_index]

    # 对角度进行限制，主要是避免激光数据抖动产生的影响
    angle = max_index - 90 if abs(max_index - 90) > 15 else 0
    angle = angle * np.pi / 180
    # speed = 3.5
    speed = speed_param - P_param * abs(angle)
    
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=angle
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    scan_sub = rospy.Subscriber('/scan', LaserScan, disparity_extender_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
