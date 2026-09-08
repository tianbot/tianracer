#!/usr/bin/env python3
# Created by Chen Yuxuan
# Modified by Tian Bo
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from math import cos, sin

robot_name = "tianracer"
marker_frame = "tianracer/base_footprint"
FILTER_VALUE = 10.0
THETA = np.deg2rad(60.0)
TARGET_DIS = 0.6
LOOK_AHEAD_DIS = 0.7
STEERING_GAIN = 0.8
CORNER_STEERING_GAIN = 0.6
FRONT_STEERING_GAIN = 0.7
RIGHT_OPENING_DISTANCE = 1.0
RIGHT_OPENING_DELTA = 0.25
RIGHT_OPENING_GAIN = 0.9
RIGHT_OPENING_FRONT_DISTANCE = 1.2
RIGHT_OPENING_RIGHT_FRONT_MAX = 1.5
LEFT_OPENING_DISTANCE = 1.0
LEFT_OPENING_DELTA = 0.25
LEFT_OPENING_GAIN = 0.9
LEFT_OPENING_FRONT_DISTANCE = 1.2
LEFT_OPENING_LEFT_FRONT_MAX = 1.5
OPENING_SWITCH_MARGIN = 0.2
WALL_SWITCH_DISTANCE_MARGIN = 0.25
CRUISE_SPEED = 0.6
SLOW_SPEED = 0.3
CRAWL_SPEED = 0.15
MAX_STEERING_ANGLE = 0.5
STEERING_SLOWDOWN_ANGLE = 0.22
FRONT_SLOW_DISTANCE = 0.9
FRONT_STOP_DISTANCE = 0.3
last_steering_angle = 0.0
last_follow_side = "right"
FOLLOW_MODE = "right_wall"

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
    m.header.frame_id = marker_frame
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
    m.header.frame_id = marker_frame
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

def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    index = int((angle - data.angle_min) / data.angle_increment)
    if index < 0 or index >= len(data.ranges):
        return FILTER_VALUE

    dis = data.ranges[index]
    if not np.isfinite(dis) or dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis


def compute_wall_steering(side_distance, front_side_distance, side_sign):
    if side_distance >= FILTER_VALUE or front_side_distance >= FILTER_VALUE:
        return 0.0, False

    denominator = front_side_distance * np.sin(THETA)
    if abs(denominator) <= 1e-6:
        return 0.0, False

    alpha = np.arctan(
        (front_side_distance * np.cos(THETA) - side_distance) / denominator
    )
    projected_distance = side_distance * np.cos(alpha) + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_distance
    steering = side_sign * STEERING_GAIN * error

    corner_tightening = max(0.0, side_distance - front_side_distance)
    steering += side_sign * CORNER_STEERING_GAIN * corner_tightening
    return steering, True


def wall_following_callback(data):
    """
    Implements simple wall following at
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
    """
    global last_steering_angle, last_follow_side

    # naming convention according to above pdf
    right_distance = get_range(data, -90)
    right_front_distance = get_range(data, -90 + np.rad2deg(THETA))
    left_distance = get_range(data, 90)
    left_front_distance = get_range(data, 90 - np.rad2deg(THETA))
    ft_dis = get_range(data, 0)
    steering_angle = last_steering_angle
    right_opening_detected = (
        ft_dis < RIGHT_OPENING_FRONT_DISTANCE
        and right_distance > RIGHT_OPENING_DISTANCE
        and (right_distance - right_front_distance) > RIGHT_OPENING_DELTA
        and right_front_distance < RIGHT_OPENING_RIGHT_FRONT_MAX
    )
    left_opening_detected = (
        ft_dis < LEFT_OPENING_FRONT_DISTANCE
        and left_distance > LEFT_OPENING_DISTANCE
        and (left_distance - left_front_distance) > LEFT_OPENING_DELTA
        and left_front_distance < LEFT_OPENING_LEFT_FRONT_MAX
    )
    right_opening_strength = max(0.0, right_distance - right_front_distance)
    left_opening_strength = max(0.0, left_distance - left_front_distance)

    right_steering, right_valid = compute_wall_steering(
        right_distance, right_front_distance, 1.0
    )
    left_steering, left_valid = compute_wall_steering(
        left_distance, left_front_distance, -1.0
    )
    prefer_right_wall = (
        right_valid
        and (
            right_distance + WALL_SWITCH_DISTANCE_MARGIN < left_distance
            or right_front_distance + WALL_SWITCH_DISTANCE_MARGIN < left_front_distance
        )
    )
    prefer_left_wall = (
        left_valid
        and (
            left_distance + WALL_SWITCH_DISTANCE_MARGIN < right_distance
            or left_front_distance + WALL_SWITCH_DISTANCE_MARGIN < right_front_distance
        )
    )

    follow_side = last_follow_side
    if FOLLOW_MODE == "left_wall":
        follow_side = "left"
    elif FOLLOW_MODE == "right_wall":
        follow_side = "right"
    else:
        if (
            right_opening_detected
            and left_opening_detected
            and (right_opening_strength - left_opening_strength) > OPENING_SWITCH_MARGIN
            and left_valid
        ):
            follow_side = "left"
        elif (
            right_opening_detected
            and left_opening_detected
            and (left_opening_strength - right_opening_strength) > OPENING_SWITCH_MARGIN
            and right_valid
        ):
            follow_side = "right"
        elif right_opening_detected and not left_opening_detected and left_valid:
            follow_side = "left"
        elif left_opening_detected and not right_opening_detected and right_valid:
            follow_side = "right"
        elif follow_side == "right" and not right_valid and left_valid:
            follow_side = "left"
        elif follow_side == "left" and not left_valid and right_valid:
            follow_side = "right"
        elif follow_side == "left" and prefer_right_wall and not prefer_left_wall:
            follow_side = "right"
        elif follow_side == "right" and prefer_left_wall and not prefer_right_wall:
            follow_side = "left"
        elif follow_side not in ("left", "right"):
            follow_side = "right" if right_valid else "left"

    if follow_side == "left" and left_valid:
        steering_angle = left_steering
    elif follow_side == "right" and right_valid:
        steering_angle = right_steering
    elif right_valid:
        follow_side = "right"
        steering_angle = right_steering
    elif left_valid:
        follow_side = "left"
        steering_angle = left_steering
    else:
        steering_angle = 0.0

    opening_detected = right_opening_detected or left_opening_detected
    if ft_dis < FRONT_SLOW_DISTANCE and not opening_detected:
        front_sign = 1.0 if follow_side == "right" else -1.0
        steering_angle += front_sign * FRONT_STEERING_GAIN * (FRONT_SLOW_DISTANCE - ft_dis)

    steering_angle = float(np.clip(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE))
    last_steering_angle = steering_angle
    last_follow_side = follow_side

    speed = CRUISE_SPEED
    if (not right_valid and not left_valid) or abs(steering_angle) > STEERING_SLOWDOWN_ANGLE:
        speed = min(speed, SLOW_SPEED)

    if ft_dis < FRONT_STOP_DISTANCE:
        # Ackermann car cannot pivot in place. Keep a small crawl speed when
        # already steering, otherwise it gets stuck with speed=0 and wheels turned.
        if opening_detected or abs(steering_angle) > 0.05:
            speed = CRAWL_SPEED
        else:
            speed = 0.0
    elif ft_dis < FRONT_SLOW_DISTANCE:
        speed = min(speed, SLOW_SPEED)

    # Local visualization in robot frame keeps the node fully reactive.
    if follow_side == "left":
        angle_side = np.deg2rad(90.0)
        angle_front_side = angle_side - THETA
        side_range = 0.0 if left_distance >= FILTER_VALUE else left_distance
        front_side_range = 0.0 if left_front_distance >= FILTER_VALUE else left_front_distance
    else:
        angle_side = np.deg2rad(-90.0)
        angle_front_side = angle_side + THETA
        side_range = 0.0 if right_distance >= FILTER_VALUE else right_distance
        front_side_range = 0.0 if right_front_distance >= FILTER_VALUE else right_front_distance

    b_x = side_range * cos(angle_side)
    b_y = side_range * sin(angle_side)
    a_x = front_side_range * cos(angle_front_side)
    a_y = front_side_range * sin(angle_front_side)

    point_scale = Vector3(0.2, 0.2, 0.2)  # Size of the point
    point_b = display_point([b_x, b_y, 0.0], point_scale, 1, (1.0, 0.0, 0.0))  # Point b is red
    point_a = display_point([a_x, a_y, 0.0], point_scale, 2, (0.0, 0.0, 1.0))  # Point a is blue

    arrow_scale = Vector3(0.05, 0.2, 0.2)  # Size of the arrow
    direction_length = 1.0  # Length of the arrow
    arrow = display_direction(
        arrow_scale,
        Point(0.0, 0.0, 0.0),
        Point(direction_length * cos(steering_angle),
              direction_length * sin(steering_angle),
              0.0),
        3
    )

    point_b_pub.publish(point_b)
    point_a_pub.publish(point_a)
    arrow_pub.publish(arrow)

    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = rospy.Time.now()
    drive_msg.drive.steering_angle = steering_angle
    angle = steering_angle * 180 / np.pi
    drive_msg.drive.speed = speed
    rospy.loginfo_throttle(
        1.0,
        "wall_following side=%s speed=%.2f steering=%.2f rad (%.1f deg) front=%.2f right=%.2f right_front=%.2f left=%.2f left_front=%.2f opening_r=%d opening_l=%d opening_sr=%.2f opening_sl=%.2f",
        follow_side,
        speed,
        steering_angle,
        angle,
        ft_dis,
        right_distance,
        right_front_distance,
        left_distance,
        left_front_distance,
        1 if right_opening_detected else 0,
        1 if left_opening_detected else 0,
        right_opening_strength,
        left_opening_strength,
    )
    drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        rospy.init_node("wall_following")

        robot_name = rospy.get_param("~robot_name", os.getenv("TIANRACER_NAME", "tianracer"))
        marker_frame = rospy.get_param("~marker_frame", robot_name + "/base_footprint")
        FOLLOW_MODE = rospy.get_param("~mode", "right_wall")
        if FOLLOW_MODE not in ("right_wall", "left_wall", "auto_switch"):
            rospy.logwarn("Unknown wall following mode '%s', fallback to right_wall", FOLLOW_MODE)
            FOLLOW_MODE = "right_wall"
        FILTER_VALUE = rospy.get_param("~range_filter_value", 10.0)
        THETA = np.deg2rad(rospy.get_param("~theta_deg", 60.0))
        TARGET_DIS = rospy.get_param("~target_distance", 0.6)
        LOOK_AHEAD_DIS = rospy.get_param("~look_ahead_distance", 0.7)
        STEERING_GAIN = rospy.get_param("~steering_gain", 0.8)
        CRUISE_SPEED = rospy.get_param("~speed", 0.5)
        MAX_STEERING_ANGLE = rospy.get_param("~max_steering_angle", 0.5)

        scan_sub = rospy.Subscriber('scan', LaserScan, wall_following_callback, queue_size=1)
        drive_pub = rospy.Publisher('ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1)

        arrow_pub = rospy.Publisher('direction_arrow', Marker, queue_size=1)
        point_a_pub = rospy.Publisher('point_a', Marker, queue_size=1)
        point_b_pub = rospy.Publisher('point_b', Marker, queue_size=1)

        rospy.loginfo(
            "Wall following node initialized: mode=%s marker_frame=%s speed=%.2f max_steer=%.2f",
            FOLLOW_MODE,
            marker_frame,
            CRUISE_SPEED,
            MAX_STEERING_ANGLE,
        )
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
