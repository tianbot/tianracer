#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo
# Created from the original wall_following file and modified to follow_the_gap by Lr-2002
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point,Vector3, Quaternion, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion # radi
from math import cos, sin
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt 

"""
todo
1. fix the problem to let the car avoide the collision
2. avoide the re-turn to the back
"""

def make_arrow_points_marker(scale, tial, tip, idnum):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id="map"
    m.header.stamp = rospy.Time.now()
    m.ns = "points_arrow"
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.y = 0 
    m.pose.orientation.w = 0
    m.scale = scale
    m.color.r = 1
    m.color.g = 0.2
    m.color.b = 0.1
    m.color.a = 0.2
    m.points = [tial, tip]
    return m


def make_round(scale, points, idnum):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id="map"
    m.header.stamp = rospy.Time.now()
    m.ns = "round"
    m.id = idnum
    m.type = Marker.CYLINDER
    m.pose.orientation.y = 0.0
    m.pose.orientation.x = 0.0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 0.707
    m.pose.position.x = points[0]
    m.pose.position.y = points[1]
    m.pose.position.z = points[2]
    m.scale = scale
    m.color.r = 0
    m.color.g = 0.
    m.color.b = 1
    m.color.a = 0.1
    # m.points = [tial, tip]
    return m

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def get_gap(dis_list, threshold):
    lis = [1 if a > threshold else 0 for a in dis_list]
    tmp = 0
    ans_list = []
    for i in lis:
        if i == 0 :
            tmp = 0
            ans_list.append(0)
        else:
            tmp +=1 
            ans_list.append(tmp)
    return ans_list

gx, gy, gz = 0, 0 ,0
gr, gp, gyaw = 0, 0 ,0
g_ang = 0

def get_pose(data):
    global gx, gy, gz, gr, gp, gyaw
    gx = data.pose.pose.position.x
    gy = data.pose.pose.position.y
    gz = data.pose.pose.position.z
    ori = [
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    ]
    (gr, gp, gyaw) = euler_from_quaternion(ori)

xx = [i - 60 for i in range(112)]
def follow_the_gap_callback(data):
    global gx, gy ,gz
    start_point = -60
    INTERVAL = 112
    end_point = start_point + INTERVAL
    STEP = 1
    threshold = 1.1
    dis_list =[]
    for ang in range(start_point, end_point, STEP):
        dis_list.append(get_range(data, ang))
    # rospy.loginfo()
    gap_list = get_gap(dis_list, threshold)
    print(dis_list)

    print(gap_list)
    end_idx = gap_list.index(max(gap_list))
    print('end_idx', end_idx)
    start_idx = end_idx - max(gap_list) + 1
    target_angle = (end_idx + start_idx) /2 *STEP + start_point
    avoidance_angle = 6 
    if target_angle > 0:
        target_angle -= avoidance_angle
    elif target_angle < 0:
        target_angle += avoidance_angle
    
    P = -1
    global gyaw

    steering_angle = target_angle  * P 
    clip_threhold = 5
    steering_angle = steering_angle if steering_angle < clip_threhold else clip_threhold
    steering_angle = steering_angle if steering_angle > -clip_threhold else -clip_threhold
    global g_ang 
    # filt_rate = 0.2
    # g_ang = g_ang * filt_rate + steering_angle * (1-filt_rate)
    # steering_angle = g_ang
    # steering_angle = steering_angle if steering_angle < clip_threhold else clip_threhold
    # steering_angle = steering_angle if steering_angle > -clip_threhold else -clip_threhold
    dis_ml = Float64MultiArray(data=dis_list)
    frame_pub.publish(dis_ml)
    speed = 1.7
    length = 1  
    gap_angle = target_angle / 180 * 3.14
    ref_yaw = gyaw + gap_angle
    print(ref_yaw, steering_angle)  
    # steering_angle = 10
    # ref_yaw = steering_angle
    arrow_pub.publish(make_arrow_points_marker(scale, Point(gx,gy,gz), Point(gx + length * cos(ref_yaw),gy + length * sin(ref_yaw) , gz), 3))
    round_pub.publish(make_round(Vector3(threshold, threshold, 0.1), (gx, gy, gz), 4))
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle 
    drive_msg.drive.speed=speed
    drive_pub.publish(drive_msg)





if __name__ == '__main__': 
  try:
    rospy.init_node("follow_the_gap")
    arrow_pub = rospy.Publisher("visualization_marker_show_orientation", Marker, queue_size=10)
    round_pub = rospy.Publisher("round_show", Marker, queue_size=10)
    scale = Vector3(0.05,0.2,0.2)
    pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_pose)
    scan_sub = rospy.Subscriber('/scan', LaserScan, follow_the_gap_callback)
    drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
    frame_pub = rospy.Publisher('/image', Float64MultiArray,queue_size=150)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass


