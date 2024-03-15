#!/usr/bin/env python
import rospy
import math
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

# Flag for emergency brake mode
emergency_brake_active = False

def set_throttle_steer(data):
    global emergency_brake_active

    pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)
    
    if not emergency_brake_active:
        
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
        
        v_lr = v_rr = v_lf = v_rf =  throttle

    # Check if emergency brake is active
    else:
        # Set all wheel velocities and steering angles to zero for emergency brake
        v_lr = v_rr = v_lf = v_rf = 0.0
        left_steer = right_steer = 0.0

    # Publish calculated or emergency brake values
    pub_vel_left_rear_wheel.publish(v_lr)
    pub_vel_right_rear_wheel.publish(v_rr)
    pub_vel_left_front_wheel.publish(v_lf)
    pub_vel_right_front_wheel.publish(v_rf)
    pub_pos_left_steering_hinge.publish(left_steer)
    pub_pos_right_steering_hinge.publish(right_steer)

def emergency_brake(req):
    global emergency_brake_active
    emergency_brake_active = True
    rospy.logwarn("Emergency brake activated!")
    return EmptyResponse()

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)
    rospy.Subscriber("ackermann_cmd_stamped", AckermannDriveStamped, set_throttle_steer)

    # Advertise emergency brake service
    rospy.Service('emergency_brake', Empty, emergency_brake)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass