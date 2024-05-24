# Set some defaults to tianbot racecar launch environment. This is not designed for users to modify.

: ${TIANRACER_BASE:=compact}  #compact, standard, fullsize, customized. 
: ${TIANRACER_BASE_PORT:=/dev/tianbot_racecar}  # /dev/ttyUSB1
: ${TIANRACER_BASE_BAUDRATE:=460800}  # 115200, 460800 
: ${TIANRACER_STEERING_DIRECTION:=normal} # normal, reverse
: ${TIANRACER_THROTTLE_DIRECTION:=normal} # normal, reverse
: ${TIANRACER_BRIDGE:=disabled} # enabled,disabled 
: ${TIANRACER_RGBD_CAMERA:=none} #realsense_d415, realsense_d435, astra, astra_pro, xtion, none 
: ${TIANRACER_RGBD_CAMERA_MODEL:=none} #d415,d435i,d455 for realsense_d. dabai, genimi for orbbec. 
: ${TIANRACER_VIDEO_DEVICE:=none} # /dev/video0 for most of the usb_cams.If you want init usb_cam in jet_cam, this should be set to none.
#/dev/video2 if using realsense, none
: ${TIANRACER_LIDAR:=rplidar_a2} # rplidar, velodyne_vlp16, osight, rslidar, hokuyo, sick, richbeam, etc..
: ${TIANRACER_LIDAR_MODEL:=a2} # a1, a2, a3, iexxx, vlp16, etc...
: ${TIANRACER_LIDAR_PORT:=/dev/tianbot_rplidar} # /dev/ttyUSB0 
: ${TIANRACER_LIDAR_IP:=192.168.198.2} # osight 192.168.10.1 richbeam 192.168.198.2 rslidar 192.168.1.200
: ${TIANRACER_GPS:=none} # none, nmea0183
: ${TIANRACER_JOY_MODE:=dji_dt7} # dji, logitech joy mode X, D


#Exports
export TIANRACER_BASE
export TIANRACER_BASE_PORT
export TIANRACER_BASE_BAUDRATE
export TIANRACER_STEERING_DIRECTION
export TIANRACER_THROTTLE_DIRECTION
export TIANRACER_BRIDGE
export TIANRACER_RGBD_CAMERA
export TIANRACER_RGBD_CAMERA_MODEL
export TIANRACER_VIDEO_DEVICE
export TIANRACER_LIDAR
export TIANRACER_LIDAR_MODEL
export TIANRACER_LIDAR_PORT
export TIANRACER_LIDAR_IP
export TIANRACER_GPS
export TIANRACER_JOY_MODE
