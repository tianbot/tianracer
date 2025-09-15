#!/usr/bin/env bash

source /home/ubuntu/tianbot_ros_ws/install/setup.bash

export ROS_DOMAIN_ID=2016

ros2 launch tianracer_bringup tianracer_bringup.launch
