#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/tianracer/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/tianracer/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/tianracer/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/tianracer/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/tianracer/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/tianracer/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    # calculate the throttle (ackermann speed) from joint angular velocity
    # wheel radius is 0.032 m
    # if the linear velocity is 1 m/s, the joint angular velocity is 1/0.032 = 31.25 rad/s
    throttle = data.drive.speed*31.25
    steering_angle = data.drive.steering_angle 
    # calculate the right and left steering angle. 
    wheelbase = 0.265
    track_width = 0.13 # it is the distance between the two front hinges.
    tan_steer = math.tan(steering_angle)
    inner_steer = math.atan2(wheelbase*tan_steer, wheelbase-track_width*tan_steer/2)
    outer_steer = math.atan2(wheelbase*tan_steer, wheelbase+track_width*tan_steer/2)
    
    if steering_angle > 0:
        left_steer = inner_steer
        right_steer = outer_steer
    else:
        left_steer = outer_steer
        right_steer = inner_steer

    # The wheel speed difference can be neglected.
    # the wheel speed is the same as the throttle.
    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(left_steer)
    pub_pos_right_steering_hinge.publish(right_steer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/tianracer/ackermann_cmd_stamped", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
