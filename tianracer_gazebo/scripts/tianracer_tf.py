#!/usr/bin/env python

import rospy

import math
import tf
from tf import transformations
import geometry_msgs.msg
from tf import broadcaster
from dynamic_reconfigure.server import Server
from abc_swarm.cfg import tf_pidConfig
import numpy as np
from simple_pid import PID


pid_linear = PID(2, 0.0, 0.0)
pid_linear.output_limits = (-0.5, 0.5)
pid_angular = PID(5, 0.0, 0.0)
pid_angular.output_limits = (-1.0, 1.0)

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {linear_kp}, {linear_ki}, {linear_kd}, {angular_kp}, {angular_ki}, {angular_kd}""".format(**config))
    pid_linear.tunings = [float(config.get(key)) for key in ['linear_kp', 'linear_ki', 'linear_kd']]
    pid_angular.tunings = [float(config.get(key)) for key in ['angular_kp','angular_ki','angular_kd']]
    return config


if __name__ == '__main__':
    rospy.init_node('tianracer_tf_listener')
    target_frame = rospy.get_param('~target_frame')
    follower_robot_name = rospy.get_param('~follower_robot_name')
    set_distance = rospy.get_param('~set_distance')

    listener = tf.TransformListener()

    follower_vel = rospy.Publisher(follower_robot_name + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    #pid_linear.sample_time = pid_angular.sample_time = 0.1
    srv = Server(tf_pidConfig, callback)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(follower_robot_name+'/base_link', target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        dis = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        angular = pid_angular(-math.atan2(trans[1],trans[0]))
        linear =  pid_linear(set_distance - dis)

        if abs(linear) < 0.02:
            linear = 0

        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        follower_vel.publish(msg)

        rate.sleep()
