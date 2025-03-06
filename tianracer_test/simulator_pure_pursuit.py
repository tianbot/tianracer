#!/usr/bin/env python
# -*- coding: utf-8 -*
"""
Referenced: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
       And: https://github.com/LejunJiang

Include file names : simulator_pure_pursuit.py(in scripts) 、data.csv(in scripts)
Running it in siumulator of F1TENTH

1、roslaunch f1tenth_simulator simulator.launch
2、rosrun tianracer_navigation simulator_pure_pursuit in your workspace

Created by Ju Yuting
reference: https://blog.csdn.net/quentinmen/article/details/104359557
"""

from datetime import date

from numpy.lib.function_base import angle
from genpy.rostime import Time
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import numpy as np
from math import pi,sqrt,atan2,sin  
import sys
import pandas as pd  
import rospy

Kp = 1.0  # 速度P控制器系数


class SubAndPub: 

    # 初始化公有变量
    def __init__(self):
        rospy.init_node("pure_pursuit_py", anonymous=True)
        self.pub_env = rospy.Publisher('env_viz',Marker,queue_size=10)
        self.sub_odom = rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.pub_dynamic = rospy.Publisher('dynamic_viz',Marker,queue_size=10)
        self.pub_drive = rospy.Publisher('ackermann_cmd_stamped',AckermannDriveStamped,queue_size=10)

        self.flag = True
        self.marker=Marker()
                        
        # 初始化marker
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD   
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.12
        self.marker.scale.y = 0.12
        self.marker.color.a = 1.0 
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0   

        self.xes=list() 
        self.yes=list()
        self.headings=list()

        # 读取数据之后存入列表，并显示为self.marker.points
        datas = pd.read_csv("data.csv",names=["x", "y", "heading"])      
        print(self.xes)
        #print(datas["x"][1])
        print(datas["x"][3])
        self.xes = datas["x"]   
        self.yes = datas["y"]
        self.headings = datas['heading']
        
        
        points = Point()
        points.x=self.xes
        points.y=self.yes
        points.z=0.0
        print(points)

        # for x in self.xes:
        #     points.x=x
        # for y in self.yes:
        #     points.y=y
        # for h in self.headings:
        #     points.z=h    
     
        self.marker.points.append(points)
        self.pub_env.publish(self.marker)

    # 写计算公式、动态marker发布、PID控制
    def odom_callback(self,data):

        # 获取当前x,y,heading
        self.x_current = data.pose.pose.position.x 
        self.y_current = data.pose.pose.position.y
        siny_cosp = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
        cosy_cosp = 1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z)
        heading_current = atan2(siny_cosp, cosy_cosp)


        # 计算公式 
        if self.flag:
            shortest_distance = 100.0
            j=0
            for i in self.xes:    
                # print(i[j])
                # print(test[j][j])
                if ((i - self.x_current) * (i - self.x_current) + (self.yes[j] - self.y_current) * (self.yes[j] - self.y_current) < shortest_distance):                    
                    shortest_distance = (i - self.x_current) * (i - self.x_current) + (self.yes[j] - self.y_current) * (self.yes[j] - self.y_current)
                j=j+1  

        Lookahead_dist = 1  
        k=0
        while (sqrt((self.xes[k] - self.x_current) * (self.xes[k] - self.x_current) + (self.yes[k] - self.y_current) * (self.yes[k] - self.y_current)) < Lookahead_dist): 
            k=k+1
            if (k > (len(self.xes) - 2)):
                k = 0
    
        real_distance = sqrt((self.xes[k] - self.x_current) * (self.xes[k] - self.x_current) + (self.yes[k] - self.y_current) * (self.yes[k] - self.y_current))
        lookahead_angle = atan2(self.yes[k] - self.y_current, self.xes[k] - self.x_current)
        del_y = real_distance * sin(lookahead_angle - heading_current)
        angle = 1 * 2.00 * del_y / (real_distance * real_distance)


        # 订阅odom后发布动态标记
        points = Point()
        marker_dy = Marker()
        points.x = self.xes[k]
        points.y = self.yes[k]
        points.z = 0.0
        print("aa")


        marker_dy.points.append(points)

        marker_dy.header.frame_id = "map"
        marker_dy.header.stamp = rospy.Time.now()
        marker_dy.id = 0
        marker_dy.type = Marker.POINTS
        marker_dy.action = Marker.ADD   
        marker_dy.pose.position.x = 0.0
        marker_dy.pose.position.y = 0.0
        marker_dy.pose.position.z = 0.0
        marker_dy.pose.orientation.x = 0.0
        marker_dy.pose.orientation.y = 0.0
        marker_dy.pose.orientation.z = 0.0
        marker_dy.pose.orientation.w = 1.0
        marker_dy.scale.x = 0.12
        marker_dy.scale.y = 0.12
        marker_dy.color.a = 1.0 
        marker_dy.color.r = 1.0
        marker_dy.color.g = 0.0
        marker_dy.color.b = 0.0   

        self.reactive_control()    #调用函数
        self.pub_env.publish(self.marker)
        self.pub_dynamic.publish(marker_dy)

    def reactive_control(self):
        ackermann_drive_cmd = AckermannDriveStamped()
        angle = ackermann_drive_cmd.drive.steering_angle
        if abs(angle) > 20.0 / 180.0 * pi:
            ackermann_drive_cmd.drive.speed = 0.5
        elif abs(angle) > 10.0 / 180.0 * pi:
            ackermann_drive_cmd.drive.speed = 1.0
        else:
            ackermann_drive_cmd.drive.speed = 1.5
        self.pub_drive.publish(ackermann_drive_cmd)
    

if __name__ == '__main__':
    try:
        sab=SubAndPub()
        rospy.spin()           
    except rospy.ROSInterruptException:
        pass
