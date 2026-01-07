#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

# 全局变量
current_v = 0.0
current_steering = 0.0
has_received_data = False

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  # 1. 如果没有角速度输入，转角必为 0
  if omega == 0:
    return 0.0
    
  # 2. 核心数学转换：增加速度为0时的保护
  if v == 0:
    v_eff = 0.1 # 有角速度但没线速度时，给定一个有效线速度计算转角
  else:
    v_eff = v
    
  # 3. 计算基础转角并乘以放大因子 gain
  steering = math.atan(wheelbase * omega / v_eff) # 保持弧度输出
  return steering

def cmd_callback(data):
  global wheelbase, current_v, current_steering, has_received_data
  
  has_received_data = True
  v = data.linear.x
  omega = data.angular.z
  
  # 计算转角
  current_steering = convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase)
  
  # 速度阈值钳位逻辑
  threshold = 0.5
  if 0 < v < threshold:
    v = threshold
  elif -threshold < v < 0:
    v = -threshold
  current_v = v

def timer_callback(event):
  """以固定 50Hz 频率发布"""
  global pub, current_v, current_steering, has_received_data
  
  if has_received_data:
    msg = AckermannDrive()
    msg.speed = current_v
    msg.steering_angle = current_steering
    pub.publish(msg)

if __name__ == '__main__': 
  try:
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    # 参数获取
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', 'cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', 'ackermann_cmd')
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    
    
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

    # --- 50Hz 定时器 ---
    rospy.Timer(rospy.Duration(1.0/50.0), timer_callback)
    
    rospy.loginfo("Node started at 50Hz. Wheelbase: %f", wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass