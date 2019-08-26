# Set some defaults to tianbot racecar launch environment

: ${TIANRACER_BASE:=compact}  #compact, standard, fullsize.
: ${TIANRACER_BASE_PORT:=/dev/tianbot_racecar}  # /dev/ttyUSB1
: ${TIANRACER_STEERING_REVERSE:=normal} # normal, reverse
: ${TIANRACER_THROTTLE_REVERSE:=normal} # normal, reverse
: ${TIANBOT_RGBD_CAMERA:=realsense_d415} #realsense_d415, realsense_d435, astra, astra_pro, xtion 
: ${TIANBOT_LIDAR:=rplidar_a2} # rplidar_a1, rplidar_a2, rplidar_a3, velodyne_vlp16
: ${TIANBOT_LIDAR_PORT:=/dev/tianbot_lidar} # /dev/ttyUSB0 
: ${TIANBOT_GPS:=none} # none, nmea0183
: ${TIANBOT_JOY:=D} # X, D


#Exports
export TIANRACER_BASE
export TIANRACER_BASE_PORT
export TIANRACER_STEERING_REVERSE
export TIANRACER_THROTTLE_REVERSE
export TIANBOT_RGBD_CAMERA
export TIANBOT_LIDAR
export TIANBOT_LIDAR_PORT
export TIANBOT_GPS
export TIANBOT_JOY
