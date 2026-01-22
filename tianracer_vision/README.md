
# tianracer_vision

视觉处理功能包，目前主要包含基于 OpenCV 的巡线功能。

## 节点

### line_follower

该节点通过订阅摄像头图像，识别特定颜色的线条并计算偏差，发布速度控制指令实现巡线。

#### 订阅话题 (Subscribed Topics)

- `camera/image_raw` ([sensor_msgs/Image](sensor_msgs/Image)): 原始摄像头图像。

#### 发布话题 (Published Topics)

- `cmd_vel` ([geometry_msgs/Twist](geometry_msgs/Twist)): 机器人运动控制指令。
- `camera/process_image` ([sensor_msgs/Image](sensor_msgs/Image)): 处理后的图像（显示检测到的中心点），用于调试。

#### 参数 (Parameters)

- `line_color` (string, 默认: `black`): 要跟踪的线条颜色。支持的可选值：
  - `yellow`
  - `red`
  - `green`
  - `blue`
  - `white`
  - `black`

## 使用方法

### 启动巡线节点

使用默认参数运行（识别黑色线）：

```bash
ros2 run tianracer_vision line_follower
```

运行并指定识别红色线，且使用命名空间（例如在 `tianracer` 下）：

```bash
ros2 run tianracer_vision line_follower --ros-args -r __ns:=/tianracer -p line_color:=red
```
