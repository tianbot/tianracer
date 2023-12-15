#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo
# @Time: 2023/10/20 17:02:12
# @Author: Jeff Wang(Lr_2002)
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
2. avoide the U-turn to the back
"""

def display_direction(scale, tail, tip, idnum):
    """
    generate arrow makers
    scale: the scale of the marker
    tail: which side has no arrow
    tip: the side has arrow
    idnum: the num of the arrow
    """
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
    m.points = [tail, tip]
    return m


def display_threshold(scale, points, idnum):
    """
    generate the round (using cylinder to generate)
    scale: the size of the round(in Vector3)
    points: the position of the marker
    idnum: the idnum 
    """
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
    return m

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    """
    from data(recv from the sub) to get the specific angle data
    data: from the sub
    angle: the angle you want to get, usually in deg
    deg: whether your angle is deg 
    """
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def get_gap(dis_list, threshold):
    """
    from the distance_list to get the gap 
    dis_list: distance_list, generated from 'get_range'
    threshold: the threshold to filter the gap
    """
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
    """
    read the position of the model
    data: recv from the sub
    """
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

INTERVAL = 112 # total interval
scan_interval = [i - 60 for i in range(INTERVAL)] # the interval you want to scan

def follow_the_gap_callback(data):
    global gx, gy ,gz, INTERVAL
    start_point = -60 # from which position to scan 
    end_point = start_point + INTERVAL # the scan end point
    STEP = 1 # step_length of the scan
    threshold = 1.1 # clip threshold
    dis_list =[] # value to go through to find the gap
    for ang in range(start_point, end_point, STEP):
        dis_list.append(get_range(data, ang))

    gap_list = get_gap(dis_list, threshold)
    # get the gap list, value stands for the length of the gap
    end_idx = gap_list.index(max(gap_list)) # find the max value in gap_list and this is the biggest gap endpoint
    start_idx = end_idx - max(gap_list) + 1 # get the start point idx
    target_angle = (end_idx + start_idx) /2 *STEP + start_point # get the mid point 


    avoidance_angle = 6  # avoidance angle of the gap(usually according to the size of the model)
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
    
    
    dis_ml = Float64MultiArray(data=dis_list)
    frame_pub.publish(dis_ml)
    speed = 1.7
    length = 1  
    gap_angle = target_angle / 180 * 3.14
    ref_yaw = gyaw + gap_angle

    arrow_pub.publish(display_direction(scale, Point(gx,gy,gz), Point(gx + length * cos(ref_yaw),gy + length * sin(ref_yaw) , gz), 3))
    round_pub.publish(display_threshold(Vector3(threshold, threshold, 0.1), (gx, gy, gz), 4))
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
    pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, get_pose)
    scan_sub = rospy.Subscriber('scan', LaserScan, follow_the_gap_callback)
    drive_pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    frame_pub = rospy.Publisher('image', Float64MultiArray,queue_size=150)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass


