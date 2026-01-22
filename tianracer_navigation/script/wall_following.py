#! /usr/bin/env python3
# Created by Chen Yuxuan
# Modified by Tian Bo, Kong Liangqian
import rclpy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    """Return a valid range value for the requested angle.

    Safely converts angle to an index into LaserScan.ranges, clamps the
    index to valid bounds, and handles NaN/inf values by returning
    FILTER_VALUE.
    """
    if deg:
        angle = np.deg2rad(angle)

    # compute index and round to nearest integer
    try:
        idx = int(round((angle - data.angle_min) / data.angle_increment))
    except Exception:
        return FILTER_VALUE

    if idx < 0 or idx >= len(data.ranges):
        return FILTER_VALUE

    dis = data.ranges[idx]
    if dis is None or not math.isfinite(dis):
        return FILTER_VALUE
    if dis < data.range_min or dis > data.range_max:
        return FILTER_VALUE
    return dis


def wall_following_callback(data, drive_pub):
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

def main():
    rclpy.init()
    node = rclpy.create_node("wall_following")

    # QoS for laser and drive topics
    qos = QoSProfile(depth=10)

    drive_pub = node.create_publisher(AckermannDriveStamped, '/drive', qos)
    node.create_subscription(LaserScan, '/scan', lambda x: wall_following_callback(x, drive_pub), qos)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()