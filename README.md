[中文版说明](https://github.com/tianbot/tianracer/blob/master/README_CN.md)  

[TianRacer详细中文操作手册](http://doc.tianbot.com/tianracer)  

# Tianracer
Tianracer is a series of low cost auotonomous vehicles controlled in closed-loop, which could follow input commands more faithfully. All series Tianracers are equipped with NVIDIA development boards, e.g, Jetson Nano, Jetson TX2, Xavier, etc. Basic version Tianracer can reach a maximum speed at 3m/s and pro version can reach 10m/s.   

## Introduction
[TianRacer ROS Wiki](https://wiki.ros.org/tianracer)
Our goal is to develop a low cost autonomous racing car with not only basic mapping, localization, navigation ability but also object detection, traffic light detection, lane keeping, etc. Now Tianracer inherits AI aiblity from Jetracer.  

## Purchase from Tianbot Official Taobao Store

Tianracer Nano version is a standard platform with all the hardware and software installed, ready-to go racing car. 
 
[点击这里进入淘宝购买或咨询客服： Purchase from Taobao:](https://item.taobao.com/item.htm?id=564703378940)  

## Appreciation to HyphaROS RaceCar 
We developed the Tianracer based on Hypha racecar. Please checkout the original authors' repo for more details.
https://github.com/Hypha-ROS/hypharos_racecar

Developer:   
* HaoChih, LIN  
* KaiChun, Wu  


## Specifications 

Speed: 3m/s
Control：closed-loop speed control
Computer: Nvidia Jetson Nano Developer Kit
Chassis: Motor + TianBoard Mini + Servo
Lidar: Slamtec Rplidar A1
Camera: 1080P Fisheye Undistorted USB3.0
Remote Controller: DJI DT7
Battery: LiPo

# Instructions
## Installation

```
cd ~/catkin_ws/src/
git clone https://github.com/tianbot/tianracer.git
cd ~/catkin_ws && catkin_make
```
## Simulation
Tianracer can be simulated in [F1tenth Simulator](https://github.com/f1tenth/f1tenth_simulator).  Install the simulator first.

```
cd ~/catkin_ws/src/
git clone https://github.com/tianbot/tianracer.git
cd ~/catkin_ws && catkin_make
```

Simulate Tianracer
```
roslaunch tianracer_navigation simulator_wall_following.launch
```

## Interfacing
Tianracer can be brought up all at once, or separately.
```
roslaunch tianracer_bringup tianracer_bringup.launch
```
### Tianracer Base
```
roslaunch tianracer_core tianracer_core.launch
```

### Lidar
```
roslaunch tianracer_bringup lidar.launch
```

### RGBD Camera (if applicable)
```
roslaunch tianracer_bringup rgbd_camera.launch
```

### USB Camera
```
roslaunch tianracer_bringup usb_cam.launch
```

### GPS (if applicable)
```
roslaunch tianracer_bringup gps.launch
```

## Mapping
After bringing up the Tianracer, we provide three methods to perform slam for 2D laser.

### GMapping
```
roslaunch tianracer_slam tianracer_gmapping.launch
```
### HectorSLAM
```
roslaunch tianracer_slam tianracer_hector.launch
```
### Cartographer
```
roslaunch tianracer_slam tianracer_cartographer.launch
```
### Save the Map
Map will be saved as tianbot_office in tianracer_slam/maps/
```
roslaunch tianracer_slam map_save.launch
```

## Navigation
After saving the map, the map can be used to perform navigation.
```
roslaunch tianracer_navigation tianracer_teb_nav.launch
```
Configure running ROS across multiple machines, then launch rviz in a PC with display
```
roslaunch tianracer_rviz view_teb_planner.launch
```

# License: GPL v3  




