#! /usr/bin/env python
# Created by Chen Yuxuan
# Modified by Tian Bo
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3, Pose, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from math import cos, sin

robot_name = os.getenv("TIANRACER_NAME", "tianracer")

# global variables are used to store robot poses
gx, gy, gz = 0, 0, 0
gr, gp, gyaw = 0, 0, 0

def display_direction(scale, tail, tip, idnum):
    """
    Generate arrow marker
    scale: scale of the marker
    tail: starting point of the arrow
    tip: ending point of the arrow
    idnum: ID of the marker
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = robot_name + "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "direction_arrow"
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale = scale
    m.color.r = 1.0
    m.color.g = 0.1
    m.color.b = 0.1
    m.color.a = 0.6
    m.points = [tail, tip]
    return m

def display_point(position, scale, idnum, color=(0.0, 1.0, 0.0)):
    """
    Generate point marker for displaying distance measurement points
    position: position of the point
    scale: size of the marker
    idnum: ID of the marker
    color: color of the marker, default is green
    """
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = robot_name + "/map"
    m.header.stamp = rospy.Time.now()
    m.ns = "distance_points"
    m.id = idnum
    m.type = Marker.SPHERE
    m.pose.position.x = position[0]
    m.pose.position.y = position[1]
    m.pose.position.z = position[2]
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale = scale
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 0.8
    return m

def get_pose(data):
    """
    Get the robot's pose
    data: data received from the subscriber
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
    global gx, gy, gz, gyaw
    
    # the angle between the two laser rays
    THETA = np.pi / 180 * 60
    # target distance from wall   ********
    TARGET_DIS = 0.5
    # the distance to project the car forward
    LOOK_AHEAD_DIS = 3
    P = 1.5

    # naming convention according to above pdf
    b = get_range(data, -90)
    a = get_range(data, -90 + np.rad2deg(THETA))
    # print(f"a{a:1.1f} b{b:1.1f}")
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))   # RuntimeWarning: divide by zero encountered in double_scalars
    AB = b * np.cos(alpha)
    projected_dis = AB + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    steering_angle = P * error

    ft_dis = get_range(data, 0)
    #speed can be set to 0.5 to 3.5 m/s, 3 by default
    speed = 2
    angle_filter = steering_angle
    
    # Create and publish visualization markers
    # Calculate positions of points a and b (based on laser scan and robot pose)
    angle_b = -90 * np.pi / 180 + gyaw  # Convert to angle in global coordinate system
    angle_a = (-90 + np.rad2deg(THETA)) * np.pi / 180 + gyaw
    
    # Calculate global coordinates of points a and b
    b_x = gx + b * cos(angle_b)
    b_y = gy + b * sin(angle_b)
    a_x = gx + a * cos(angle_a)
    a_y = gy + a * sin(angle_a)
    
    # Create point markers to display points a and b
    point_scale = Vector3(0.2, 0.2, 0.2)  # Size of the point
    point_b = display_point([b_x, b_y, gz], point_scale, 1, (1.0, 0.0, 0.0))  # Point b is red
    point_a = display_point([a_x, a_y, gz], point_scale, 2, (0.0, 0.0, 1.0))  # Point a is blue
    
    # Create arrow marker to display driving direction
    arrow_scale = Vector3(0.05, 0.2, 0.2)  # Size of the arrow
    direction_length = 1.0  # Length of the arrow
    # Calculate driving direction (based on current heading and steering angle)
    direction_angle = gyaw + steering_angle
    arrow = display_direction(
        arrow_scale,
        Point(gx, gy, gz),  # Arrow start point (robot's current position)
        Point(gx + direction_length * cos(direction_angle), 
              gy + direction_length * sin(direction_angle), 
              gz),  # Arrow end point
        3
    )
    
    # Publish markers
    point_b_pub.publish(point_b)
    point_a_pub.publish(point_a)
    arrow_pub.publish(arrow)
    
    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle=steering_angle 
    angle = steering_angle * 180 / np.pi
    drive_msg.drive.speed=speed
    #rospy.loginfo(f"rad_steering_angle: {steering_angle:1.1f} steering_angle: {angle:1.1f}  speed: {speed:1.1f}")
    print("rad_steering_angle: %.1f steering_angle: %.1f  speed: %.1f" % (steering_angle, angle, speed))
    drive_pub.publish(drive_msg)

if __name__ == '__main__': 
  try:
    rospy.init_node("wall_following")
    
    # Create Publishers and Subscribers
    scan_sub = rospy.Subscriber('scan', LaserScan, wall_following_callback)
    pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, get_pose)
    drive_pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)
    
    # Create Publishers for markers
    arrow_pub = rospy.Publisher('direction_arrow', Marker, queue_size=1)
    point_a_pub = rospy.Publisher('point_a', Marker, queue_size=1)
    point_b_pub = rospy.Publisher('point_b', Marker, queue_size=1)
    
    rospy.loginfo("Wall following node initialized with visualization")
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
