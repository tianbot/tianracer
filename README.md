# Tianracer
![all series of Tianracer](https://raw.githubusercontent.com/tianbot/tianracer/master/TIANRACER.png)
[中文版说明](https://github.com/tianbot/tianracer/blob/master/README_CN.md)  
[TianRacer详细中文操作手册](http://doc.tianbot.com/tianracer)  
Tianracer is a series of low-cost autonomous vehicles ranging in size from 1:10 to 1:5, controlled in a closed-loop system, allowing them to follow input commands more faithfully. All Tianracer models are equipped with NVIDIA development boards, including Jetson Nano, Jetson TX2, Xavier, Orin, and more. The basic version of Tianracer can reach a maximum speed of 3m/s, while the pro version can achieve speeds up to 10m/s.

## Introduction
[TianRacer ROS Wiki](https://wiki.ros.org/tianracer)
Our objective is to develop an affordable autonomous racing car, the Tianracer, with capabilities extending beyond basic mapping, localization, and navigation. It will feature advanced functionalities such as object detection, traffic light detection, and lane keeping. Currently, Tianracer builds upon and enhances the AI capabilities inherited from Jetracer.

## Purchase from Tianbot Official Taobao Store

The Tianracer Nano version is a standard platform with all the hardware and software installed, a ready-to-go racing car. 
 
[点击这里进入淘宝购买或咨询客服： Purchase from Taobao:](https://item.taobao.com/item.htm?id=564703378940)  

## Appreciation to HyphaROS RaceCar and Lord-Z
We developed the Tianracer based on the Hypha racecar. Please check out the original authors' repo for more details.
https://github.com/Hypha-ROS/hypharos_racecar

Developer:   
* HaoChih, LIN  
* KaiChun, WU  

Tianracer gazebo simulation is imported from https://github.com/Lord-Z/ackermann_gazebo

Developer:
* Yuxing, ZHANG  

## Customization 

Speed: 3m/s - 10m/s 
Control：closed-loop speed control
Computer: Nvidia Jetson Nano/TX/NX/Orin Developer Kit
Chassis: BLDC Motor + TianBoard Mini + Servo
Lidar: Slamtec / Osight / Richbeam / Livox 
Camera: 1080P Fisheye Undistorted USB3.0 / RGBD Camera
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
git clone https://github.com/f1tenth/f1tenth_simulator.git
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
Configure running ROS across multiple machines, then launch RViz in a PC with a display
```
roslaunch tianracer_rviz view_teb_planner.launch
```

# License: GPL v3  




