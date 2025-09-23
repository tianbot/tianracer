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
Lidar: Richbeam 1L
Camera: 1080P Fisheye Undistorted USB3.0
Remote Controller: DJI DT7
Battery: LiPo

# Instructions
## Installation

```
cd ~/tianracer_ros2_ws/src/
git clone https://github.com/tianbot/tianracer.git -b humble-devel
cd ~/tianracer_ros2_ws && colcon build --symlink-install
```
## Simulation
Tianracer can be simulated in [F1tenth Simulator](https://github.com/f1tenth/f1tenth_simulator).  Install the simulator first.

```
cd ~/tianracer_ros2_ws/src/
git clone https://github.com/f1tenth/f1tenth_simulator.git
cd ~/tianracer_ros2_ws && colcon build --symlink-install
```

## Interfacing
Tianracer can be brought up all at once, or separately.

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py 
```
### Tianracer Base

```bash
ros2 launch tianracer_core tianracer_core.launch.py
```

### Lidar

```bash
ros2 launch tianracer_bringup lidar.launch.py 
```

### RGBD Camera (if applicable)

```bash
ros2 launch tianracer_bringup rgbd_camera.launch.py
```

### USB Camera

```bash
ros2 launch tianracer_bringup usb_cam.launch.py
```

### GPS (if applicable)

```bash
ros2 launch tianracer_bringup gps.launch.py
```

## Mapping
After bringing up the Tianracer, we provide three methods to perform slam for 2D laser.

### GMapping

```bash
ros2 launch tianracer_slam tianracer_gmapping.launch.py
```
### SLAM TOOLBOX

```bash
ros2 launch tianracer_slam tianracer_slam_toolbox.launch.py
```
### Cartographer

```bash
ros2 launch tianracer_slam tianracer_cartographer.launch.py
```

### Save the Map
Map will be saved as tianbot_office in tianracer_slam/maps/
```bash
ros2 launch tianracer_slam map_save.launch.py
```

## Navigation
After saving the map, the map can be used to perform navigation.

### NavFn (planner) + DWB (controller)
```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=navfn_dwb
```

### NavFn (planner) + TEB (controller)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=navfn_teb
```

### SMAC (planner) + Graceful (controller)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=smac_graceful
```

### Theta Star (planner) + MPPI (controller)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_mppi
```

### Theta Starn (planner) + Regulated Pure Pursuit (controller)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_rpp
```

### Theta Star (planner) + Vector Pursuit (controller)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_vector_pur
```

# License: GPL v3