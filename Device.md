# 硬件设备：

## 底盘：
$(find tianracer_core)/launch/tianracer_core.launch


## 激光雷达：
$(find tianracer_bringup)/launch/lidar.launch


# RVIZ可视化：

## 激光雷达：
$(find tianracer_rviz)/launch/view_rplidar.launch

## IMU:
$(find tianracer_rviz)/launch/view_imu.launch

## 导航：
$(find tianracer_rviz)/launch/view_amcl.launch

## 建图：
$(find tianracer_rviz)/launch/view_gmapping.launch

## 查看由雷达生成的odom里程信息：
$(find tianracer_rviz)/launch/view_laser_odom.launch


# 功能包：

## 键盘控制：
+底盘
$(find tianracer_teleop)/launch/keyboard_teleop.launch


## 手柄控制：
+底盘
$(find tianracer_teleop)/launch/joystick_teleop.launch


## 导航：
+底盘
+IMU
+激光雷达
$(find tianracer_navigation)/launch/tianracer_amcl_nav.launch

