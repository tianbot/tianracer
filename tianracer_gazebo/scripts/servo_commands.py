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
    # print('data is as below:')
    # print(data)
    # rospy.logdebug('servo commands as below')
    # rospy.logdebug(data)

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
    wheel_width = 0.18 # 0.18, 
    hinge_length = 0.025
    tan_steer = math.tan(steering_angle)
    left_steer = math.atan2(wheelbase*tan_steer, wheelbase-track_width*tan_steer/2)
    right_steer = math.atan2(wheelbase*tan_steer, wheelbase+track_width*tan_steer/2)
    
    # # calculate the linear velocity for each wheel
    # # seems no obvious difference
    
    # angular_velocity = throttle*tan_steer/wheelbase
    # if abs(steering_angle) > 0.0001:    
    #     # rear using wheel width
    #     v_lr = throttle - angular_velocity*wheel_width/2
    #     v_rr = throttle + angular_velocity*wheel_width/2
    #     # front
    #     v_lf = (wheelbase/math.sin(left_steer) - hinge_length) * angular_velocity
    #     v_rf = (wheelbase/math.sin(right_steer) + hinge_length) * angular_velocity
    # else:
    #     v_lr = v_rr = v_lf = v_rf =  throttle
    
    # print(throttle, v_lf,v_lr,v_rf,v_rr)
    
    v_lr = v_rr = v_lf = v_rf =  throttle
    # The wheel speed difference can be neglected.
    # the wheel speed is the same as the throttle.
    pub_vel_left_rear_wheel.publish(v_lr)
    pub_vel_right_rear_wheel.publish(v_rr)
    pub_vel_left_front_wheel.publish(v_lf)
    pub_vel_right_front_wheel.publish(v_rf)
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
