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

最大速度: 3m/s
控制：线性闭环控制 
处理器: Nvidia Jetson Nano 开发套件
底盘: 电机 + TianBoard Mini + 舵机
激光雷达: Richbeam 1L
摄像头: 1080P 鱼眼无畸变 USB3.0
遥控器: 大疆 DJI DT7
电池: 锂聚合物

# 使用
## 安装

```bash
cd ~/tianracer_ros2_ws/src/
git clone https://github.com/tianbot/tianracer.git -b humble-devel
cd ~/tianracer_ros2_ws && colcon build --symlink-install
```
## 启动与接口
你可以一次性启动 Tianracer 的所有功能，也可以单独启动各个组件。

```bash
ros2 launch tianracer_bringup tianracer_bringup.launch.py 
```

## 单独启动各组件

### Tianracer 底盘

```bash
ros2 launch tianracer_core tianracer_core.launch.py
```

### 激光雷达

```bash
ros2 launch tianracer_bringup lidar.launch.py 
```

### RGBD 摄像头 (若装备)

```bash
ros2 launch tianracer_bringup rgbd_camera.launch.py
```

### USB 摄像头 (若装备)
 
```bash
ros2 launch tianracer_bringup usb_cam.launch.py
```

### GPS (若装备)

```bash
ros2 launch tianracer_bringup gps.launch.py
```

## 使用 RVIZ 调试

### 查看激光雷达
```bash
ros2 launch tianracer_rviz view_lidar.launch.py
```

### 查看 IMU
```bash
ros2 launch tianracer_rviz view_imu.launch.py
```

### 查看里程计 (Odom)
```bash
ros2 launch tianracer_rviz view_odom.launch.py
```

### 查看图像
```bash
ros2 launch tianracer_rviz view_image.launch.py
```

### 查看机器人 URDF 或 TF
```bash
ros2 launch tianracer_rviz view_robot.launch.py
```

## 建图
启动 Tianracer 后，我们提供三种 2D 激光雷达建图方法。

### GMapping

```bash
ros2 launch tianracer_slam gmapping.launch.py
```

### SLAM TOOLBOX

```bash
ros2 launch tianracer_slam slam_toolbox.launch.py
```

### Cartographer

```bash
ros2 launch tianracer_slam cartographer.launch.py
```

### 查看建图过程

```bash
ros2 launch tianracer_rviz view_mapping.launch.py
```

### 保存地图
地图将默认以 `tianbot_office` 为名保存在 `tianracer_slam/maps/` 目录下。
```bash
ros2 launch tianracer_slam map_save.launch.py
```

## 响应式控制

### 自动跟墙 (Wall Following)
```bash
ros2 launch tianracer_navigation wall_following.launch.py
```

### NPU 竞速模式 1
```bash
ros2 launch tianracer_navigation npu_battle_fast1.launch.py
```

### NPU 竞速模式 2

```bash
ros2 launch tianracer_navigation npu_battle_fast2.launch.py
```

## 导航
保存地图后，即可使用该地图进行导航。

### NavFn (全局规划) + DWB (局部规划)
```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=navfn_dwb
```

### NavFn (全局规划) + TEB (局部规划)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=navfn_teb
```

### SMAC (全局规划) + Graceful (局部规划)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=smac_graceful
```

### Theta Star (全局规划) + MPPI (局部规划)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_mppi
```

### Theta Star (全局规划) + Regulated Pure Pursuit (局部规划)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_rpp
```

### Theta Star (全局规划) + Vector Pursuit (局部规划)

```bash
ros2 launch tianracer_navigation2 nav2.launch.py use_map:=tianbotoffice_603 use_planner:=theta_star_vector_pur
```

# License: GPL v3  
