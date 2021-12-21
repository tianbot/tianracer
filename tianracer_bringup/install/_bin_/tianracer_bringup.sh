#!/usr/bin/env bash

#IP=$(ip addr show eth0 | grep -w inet | awk '{print $2}' | awk -F / '{print $1}')

source /home/ubuntu/tianbot_ros_ws/install/local_setup.bash

# export ROS_MASTER_URI=http://localhost:11311

#export ROS_IP=${IP}

ros2 launch tianracer_bringup tianracer_bringup.launch
