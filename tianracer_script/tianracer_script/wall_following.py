# Created by Chen Yuxuan
# Modified by Tian Bo, Kong Liangqian
import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

FILTER_VALUE = 10.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
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

    drive_pub = node.create_publisher('/drive', AckermannDriveStamped, queue_size=1)
    node.create_subscription('/scan', LaserScan, lambda x: wall_following_callback(x, drive_pub))
    rclpy.spin(node)

if __name__ == '__main__':
    main()