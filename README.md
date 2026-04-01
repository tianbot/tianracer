# Tianracer
![all series of Tianracer](https://raw.githubusercontent.com/tianbot/tianracer/master/TIANRACER.png)
[中文版说明](https://github.com/tianbot/tianracer/blob/master/README_CN.md)  
[TianRacer详细中文操作手册](http://docs.tianbot.com/tianracer)  
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
The main simulation workflow in this repository is based on `Gazebo`.

If you want to bring up the full simulation stack first, start with:

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch
```

In this repository, launch files starting with `demo_` are usually the easiest end-to-end entry points. This one brings up:

- Gazebo world
- map server
- robot spawn
- controllers
- TEB navigation
- AMCL localization
- RViz

If you only want to check whether the simulated car, controllers, odometry, and TF are working normally, run:

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

This launch mainly starts:

- `tianracer_on_racetrack.launch`
- `tianracer_control.launch`

It is a better starting point when you only want to confirm that:

- the Gazebo world starts correctly
- the robot is spawned correctly
- the controllers are loaded
- odometry and TF are being published normally

For multi-robot simulation, start with:

```bash
roslaunch tianracer_gazebo swarm/demo_sim_two_tianracer.launch
```

If you only want two cars in Gazebo without navigation, run:

```bash
roslaunch tianracer_gazebo swarm/two_tianracer_bringup.launch
```

The related packages are organized as follows:

- `tianracer_gazebo`: simulation worlds, robot model, controllers, and Gazebo demo launches
- `tianracer_navigation`: navigation algorithms and generic navigation launches
- `tianracer_rviz`: generic RViz configurations
- `tianracer_description`: meshes and TF-related model resources

For now, the recommended simulation entries in this repository are:

- `tianracer_gazebo/launch/demo_tianracer_teb_nav.launch`
- `tianracer_gazebo/launch/tianracer_bringup.launch`
- the `tianracer_gazebo/launch/swarm/` launches

The following files should not be treated as the main simulation entry points for now:

- `tianracer_gazebo/launch/navigation.launch`
- `tianracer_gazebo/launch/tianracer_gmapping.launch`
- `tianracer_gazebo/launch/spawn_model.launch`

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
If you want to build a map in the current simulation workflow, bring up Gazebo first and confirm that the car, lidar, odometry, and TF are all normal:

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

After Gazebo is running, choose one SLAM method below.

### GMapping
This is the most direct 2D mapping entry in the current repository:

```bash
roslaunch tianracer_slam tianracer_gmapping.launch
```

### HectorSLAM
If you want to compare another 2D mapping pipeline, run:

```bash
roslaunch tianracer_slam tianracer_hector.launch
```

### Cartographer
For Gazebo, use the Gazebo-specific Cartographer launch so that `use_sim_time` is configured for simulation:

```bash
roslaunch tianracer_slam gazebo/gazebo_cartographer_2d.launch
```

The generic launch below is still available, but it is more suitable when you already know the time source and topic setup you want to use:

```bash
roslaunch tianracer_slam tianracer_cartographer.launch
```

### Save the Map
If the map is generated in Gazebo, it is better to save it with the Gazebo-specific launch. By default, the map will be written to `tianracer_gazebo/maps/` and the file prefix is `racetrack`:

```bash
roslaunch tianracer_slam gazebo/gazebo_map_save.launch
```

The generic save entry is still available and writes to `tianracer_slam/maps/` with the default prefix `tianbot_office`:

```bash
roslaunch tianracer_slam map_save.launch
```

## Navigation
If you only want to run the complete Gazebo navigation demo, the simplest entry is still:

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch
```

That launch already brings up the Gazebo world, map server, localization, navigation, and RViz together.

If you already have a map and want to run the navigation stack separately, then use the launches in `tianracer_navigation`.

### Teb Local Planner
This is the main map-based navigation entry in `tianracer_navigation`:

```bash
roslaunch tianracer_navigation tianracer_teb_nav.launch use_rviz:=true map_file:=tianbot_office
```

This launch mainly starts:

- `map_server`
- `AMCL`
- `move_base`
- `cmd_vel_to_ackermann_drive.py`
- RViz (optional)

If you are using Gazebo, bring up the simulated car first:

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

### L1_Controller Local Planner
If you want to test the L1 local planner on top of the same map/localization flow, run:

```bash
roslaunch tianracer_navigation tianracer_l1_nav.launch map_file:=tianbot_office use_rviz:=true
```

This launch keeps the same map server and AMCL path, but switches the local planning side to the L1 controller flow.

If you only want to open the generic RViz view from another machine after ROS networking is configured, run:

```bash
roslaunch tianracer_rviz view_teb_planner.launch
```

## Reactive Navigation
The repository also keeps two reactive navigation entries that are useful for quick experiments in Gazebo.

If you are testing in Gazebo, bring up the simulated car first:

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

### wall_following
Then run:

```bash
roslaunch tianracer_navigation wall_following.launch use_rviz:=true
```

This launch mainly starts:

- `wall_following.py`
- `ackermann_convert_drive.py`
- RViz (optional)

One detail is worth stating clearly: in the current repository, this launch still brings up `map_server` and `AMCL` for the existing debug view and topic organization. So it is better to treat it as a reactive driving demo rather than a fully isolated no-map stack.

### follow_the_gap
Then run:

```bash
roslaunch tianracer_navigation follow_the_gap.launch use_rviz:=true
```

This launch mainly starts:

- `tianracer_gazebo/follow_the_gap.py`
- `ackermann_convert_drive.py`
- RViz (optional)

This one has the same caveat: although it is grouped here as no-map navigation, the current launch file still includes `map_server` and `AMCL`. For now, it should also be understood as a reactive navigation demo built on top of the current debugging layout.

# License: GPL v3  




