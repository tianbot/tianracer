# Set some defaults to tianbot racecar launch environment

: ${TIANRACER_BASE:=compact}  #compact, standard, fullsize.
: ${TIANRACER_BASE_PORT:=/dev/tianbot_racecar}  # /dev/ttyUSB1
: ${TIANRACER_STEERING_DIRECTION:=normal} # normal, reverse
: ${TIANRACER_THROTTLE_DIRECTION:=normal} # normal, reverse
: ${TIANRACER_BRIDGE:=disabled} # enabled,disabled 
: ${TIANRACER_RGBD_CAMERA:=none} #realsense_d415, realsense_d435, astra, astra_pro, xtion, none 
: ${TIANRACER_VIDEO_DEVICE:=none} # /dev/video0 for most of the usb_cams.If you want init usb_cam in jet_cam, this should be set to none.
#/dev/video2 if using realsense, none
: ${TIANRACER_LIDAR:=rplidar_a2} # rplidar_a1, rplidar_a2, rplidar_a3, velodyne_vlp16
: ${TIANRACER_LIDAR_PORT:=/dev/tianbot_rplidar} # /dev/ttyUSB0 
: ${TIANRACER_GPS:=none} # none, nmea0183
: ${TIANRACER_JOY_MODE:=X} # logitech joy mode X, D


#Exports
export TIANRACER_BASE
export TIANRACER_BASE_PORT
export TIANRACER_STEERING_DIRECTION
export TIANRACER_THROTTLE_DIRECTION
export TIANRACER_BRIDGE
export TIANRACER_RGBD_CAMERA
export TIANRACER_VIDEO_DEVICE
export TIANRACER_LIDAR
export TIANRACER_LIDAR_PORT
export TIANRACER_GPS
export TIANRACER_JOY_MODE
