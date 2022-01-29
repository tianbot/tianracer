#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

x = 0

def callback(data):
    global x
    x=data.pose.pose.position.x
    # print(x)


    # y=data.pose.pose.position.y



def move3():
    
    # 设定目标点
    target = Pose(Point(-7, 0, 0.000), Quaternion(0.000, 0.000,-0.0015, 0.99))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target))


    # 向目标进发
    move_base.send_goal(goal)

    # move_base.wait_for_result()

    while x>-12.2:
        # print(x)
        pass
    # print('a')
    # move_base.cancel_goal()
    move1()





def move2():
    
    # 设定目标点
    target = Pose(Point(-7, 8.7, 0.000), Quaternion(0.000, 0.000,0.99, 0.0016))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target))


    # 向目标进发
    move_base.send_goal(goal)

    # move_base.wait_for_result()

    while x>-4.5:
        # print(x)
        pass
    print('a')
    # move_base.cancel_goal()
    move3()
    print('b')




def move1():
    
    # 设定目标点
    target = Pose(Point(7, 0, 0.000), Quaternion(0.000, 0.000,-0.0015, 0.99))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target))

    # 向目标进发
    move_base.send_goal(goal)

    # move_base.wait_for_result()

    while x<3.5:
        # print(x)
        pass
    print('a')
    # move_base.cancel_goal()
    move2()
    print('b')










if __name__ == '__main__':
    # 节点初始化
    rospy.init_node('move_test', anonymous=True)

    # 订阅move_base服务器的消息
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.Subscriber('odom', Odometry, callback,queue_size=1,buff_size=52428800)

    rospy.loginfo("Waiting for move_base action server...")

    # 等待连接服务器，5s等待时间限制
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move base server")

    for i in range(0,100):
        move1()



