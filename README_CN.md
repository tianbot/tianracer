[English](https://github.com/tianbot/tianracer/blob/master/README.md)  

[TianRacer详细中文操作手册](http://doc.tianbot.com/tianracer)  

# 天驰无人竞速车 Tianracer
Tianracer是由天之博特开发的低成本高速高仿真无人车，底盘采用闭环线性控制，具有良好的动力性能和操控性。全系列产品采用英伟达NVidia的开发板，提供强大的并行运算能力。基础款车型可以达到3m/s的速度，而专业版可达到10m/s速度。相同的软件架构可以方便进行算法开发和移植。


## 介绍
[TianRacer ROS Wiki](https://wiki.ros.org/tianracer)
我们的目标时开发一款低成本无人车，不仅能够测试建图、定位、导航等算法，同时还具有物体识别、信号灯识别、车道线保持等功能。现在，Tianracer不仅能够实现F1tenth无人竞速车的功能，还能够继承Jetracer的特性。

## 购买

基础版采用Jetson Nano控制器，包括阿克曼底盘、悬架、动力、线控、惯导、激光、视觉、编码器等等，无需繁冗的硬件开发，开机即可使用，教程和讲义丰富，方便学习以及高校无人车教学. 
 
[点击这里进入淘宝购买或咨询客服： Purchase from Taobao:](https://item.taobao.com/item.htm?id=564703378940)  

## 致谢 
2017年在ROS Summer School从台湾小帅哥林浩鋕手里接过Hypha Racecar之后，希望能把这个项目发扬光大。
https://github.com/Hypha-ROS/hypharos_racecar

开发者:   
* HaoChih, LIN  
* KaiChun, Wu  


## 参数

最大速度: 基础版3m/s，专业版10m/s
控制：线性闭环控制 
处理器: Nvidia Jetson Nano
底盘: 悬架、电机、电调、舵机
控制器：TianBoard Mini
激光雷达: Slamtec Rplidar A1
摄像头: 1080P Fisheye Undistorted USB3.0
遥控器: 大疆DJI DT7
电池: 锂聚合物

# 使用
## 安装

```
cd ~/catkin_ws/src/
git clone https://github.com/tianbot/tianracer.git
cd ~/catkin_ws && catkin_make
```
## 仿真
Tianracer可以在[F1tenth Simulator](https://github.com/f1tenth/f1tenth_simulator)中进行仿真. 首先安装仿真环境.

```
cd ~/catkin_ws/src/
git clone https://github.com/f1tenth/f1tenth_simulator.git
cd ~/catkin_ws && catkin_make
```

进行仿真
```
roslaunch tianracer_navigation simulator_wall_following.launch
```

## 通信
Tianracer可以一次全部启动,或者单独启动各个部件.
```
roslaunch tianracer_bringup tianracer_bringup.launch
```
### Tianracer底盘
```
roslaunch tianracer_core tianracer_core.launch
```

### 激光雷达
```
roslaunch tianracer_bringup lidar.launch
```

### 深度相机 (若装备)
```
roslaunch tianracer_bringup rgbd_camera.launch
```

### USB摄像头
```
roslaunch tianracer_bringup usb_cam.launch
```

### GPS (若装备)
```
roslaunch tianracer_bringup gps.launch
```

## 建图
启动Tianracer后, 我们提供三种方式进行建图.

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
### 保存地图
地图默认保存在tianracer_slam/maps/目录下，名称为tianbot_office
```
roslaunch tianracer_slam map_save.launch
```

## 导航
保存地图后，下列程序会使用默认地图进行导航.
```
roslaunch tianracer_navigation tianracer_teb_nav.launch
```
如果正确配置了ROS的多机互联, 可以在控制台电脑上打开RViz进行查看
```
roslaunch tianracer_rviz view_teb_planner.launch
```

# License: GPL v3  
