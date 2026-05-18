[English](https://github.com/tianbot/tianracer/blob/master/README.md)  

[TianRacer详细中文操作手册](http://docs.tianbot.com/tianracer)  

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
目前仓库里主要使用 `Gazebo` 做仿真。

如果你只是想先把整套仿真流程跑起来，建议直接运行：

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch
```

按照这个仓库现在的习惯，`demo` 开头的 launch 文件，一般就是把一整条链路都串好了，方便直接演示和上手。上面这个 launch 会一次启动：

- Gazebo世界
- 地图
- 车模生成
- 控制器
- TEB导航
- AMCL定位
- RViz

如果你现在还不想一上来就带导航，只是想先确认仿真车、控制器、里程计这些是不是正常，可以先运行：

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

这个 launch 主要串的是：

- `tianracer_on_racetrack.launch`
- `tianracer_control.launch`

也就是说，它更适合先看：

- Gazebo世界有没有正常起来
- 车有没有正常生成
- 控制器有没有正常加载
- odom 和 TF 是否正常

如果要看双车仿真，可以先运行：

```bash
roslaunch tianracer_gazebo swarm/demo_sim_two_tianracer.launch
```

如果只想起双车，不带导航，可以运行：

```bash
roslaunch tianracer_gazebo swarm/two_tianracer_bringup.launch
```

这里顺便说明一下几个相关功能包的分工：

- `tianracer_gazebo`：仿真世界、模型、控制器、仿真导航示例
- `tianracer_navigation`：导航算法和通用导航 launch
- `tianracer_rviz`：通用 RViz 配置
- `tianracer_description`：模型 mesh 和一些 TF 相关资源

当前写文档时，建议先围绕下面这几个入口来写：

- `tianracer_gazebo/launch/demo_tianracer_teb_nav.launch`
- `tianracer_gazebo/launch/tianracer_bringup.launch`
- `tianracer_gazebo/launch/swarm/` 下面这一组双车文件

下面这几个 launch 目前先不要当成主入口，后面再单独整理：

- `tianracer_gazebo/launch/navigation.launch`
- `tianracer_gazebo/launch/tianracer_gmapping.launch`
- `tianracer_gazebo/launch/spawn_model.launch`

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
如果现在是按仓库里这套 Gazebo 仿真来用，建议先把仿真车起起来，先确认激光、里程计和 TF 都正常，再开始建图：

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

Gazebo 起好以后，再按下面三种方式选一种来建图。

### GMapping
这是当前仓库里最直接的 2D 建图入口：

```bash
roslaunch tianracer_slam tianracer_gmapping.launch
```

### HectorSLAM
如果想换一套 2D 建图方法做对比，可以运行：

```bash
roslaunch tianracer_slam tianracer_hector.launch
```

### Cartographer
如果是在 Gazebo 里建图，建议直接用 Gazebo 这一套 Cartographer 入口，这样 `use_sim_time` 会按仿真时间配置好：

```bash
roslaunch tianracer_slam gazebo/gazebo_cartographer_2d.launch
```

仓库里也保留了通用入口：

```bash
roslaunch tianracer_slam tianracer_cartographer.launch
```

这个更适合你已经明确知道自己要接哪套时间源和话题的时候再单独用。

### 保存地图
如果地图是在 Gazebo 里建出来的，建议直接用 Gazebo 这套保存入口。默认会把地图保存到 `tianracer_gazebo/maps/`，文件名前缀默认是 `racetrack`：

```bash
roslaunch tianracer_slam gazebo/gazebo_map_save.launch
```

仓库里原来的通用保存入口也还在，默认会把地图保存到 `tianracer_slam/maps/`，文件名前缀默认是 `tianbot_office`：

```bash
roslaunch tianracer_slam map_save.launch
```

## 导航
如果只是想先把 Gazebo 里的整套导航流程直接跑起来，最省事的入口还是：

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch
```

这个 launch 会把 Gazebo 世界、地图、定位、导航和 RViz 一次带起来，适合先看整条链路是不是通的。

如果地图已经准备好了，只是想单独测试导航算法，再去用 `tianracer_navigation` 下面这几个入口。

### Teb Local Planner
这是 `tianracer_navigation` 里现在最主要的有图导航入口：

```bash
roslaunch tianracer_navigation tianracer_teb_nav.launch use_rviz:=true map_file:=tianbot_office
```

这套 launch 主要会起：

- `map_server`
- `AMCL`
- `move_base`
- `cmd_vel_to_ackermann_drive.py`
- RViz（可选）

如果是在 Gazebo 里测试，先把仿真车起起来就行：

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

### L1_Controller Local Planner
如果想在同一套地图和定位链路上切到 L1 局部规划器，可以运行：

```bash
roslaunch tianracer_navigation tianracer_l1_nav.launch map_file:=tianbot_office use_rviz:=true
```

这套入口保留了 map server 和 AMCL，只是把局部规划这一层换成了 L1 controller。

如果已经把 ROS 多机互联配置好了，只想在另一台带显示器的电脑上打开通用 RViz 视图，可以运行：

```bash
roslaunch tianracer_rviz view_teb_planner.launch
```

## 无图导航
这两套入口更适合拿来做 Gazebo 里的反应式导航测试。

如果是在 Gazebo 里跑，还是先把仿真车起起来：

```bash
roslaunch tianracer_gazebo tianracer_bringup.launch
```

### wall_following
然后运行：

```bash
roslaunch tianracer_navigation wall_following.launch use_rviz:=true
```

这套入口主要会起：

- `wall_following.py`
- `ackermann_convert_drive.py`
- RViz（可选）

这份 `wall_following` 现在按反应式演示入口组织，只依赖激光数据并发布 `ackermann_cmd_stamped`，再通过 `ackermann_convert_drive.py` 兼容现有 `ackermann_cmd` 输出链路，不再带起地图和 `AMCL`。

更准确地说，它适合当轻量反应式演示入口使用，而不是完整导航栈的直接替代。

### follow_the_gap
然后运行：

```bash
roslaunch tianracer_navigation follow_the_gap.launch use_rviz:=true
```

这套入口主要会起：

- `tianracer_gazebo/follow_the_gap.py`
- `ackermann_convert_drive.py`
- RViz（可选）

# License: GPL v3  
