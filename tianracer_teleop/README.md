# tianracer_teleop

TianRacer 遥控功能包。支持通过键盘或手柄（Joystick）对机器人进行远程控制。

## 功能特性

- **键盘控制**: 提供基于 `geometry_msgs/Twist` 的全向/差速控制。
- **手柄控制**: 支持多种手柄模式，直接发布 `ackermann_msgs/AckermannDrive` 控制指令。
- **命名空间支持**: 所有节点话题均采用相对路径，可完美适配多车场景。

## 安装与编译

在您的 ROS 2 工作空间中编译：

```bash
cd ~/tianracer_ros2_ws
colcon build --packages-select tianracer_teleop
source install/setup.bash
```

## 节点详解

### teleop_twist_keyboard

键盘控制节点。

- **发布话题**: 
  - `cmd_vel` ([geometry_msgs/Twist](geometry_msgs/Twist)): 速度控制指令。
- **参数**:
  - `speed` (double, 默认: 0.5): 初始线速度。
  - `turn` (double, 默认: 1.0): 初始角速度。

### tianracer_joy

手柄控制节点。

- **订阅话题**:
  - `joy` ([sensor_msgs/Joy](sensor_msgs/Joy)): 手柄原始数据。
- **发布话题**:
  - `ackermann_cmd` ([ackermann_msgs/AckermannDrive](ackermann_msgs/AckermannDrive)): 阿克曼转向控制指令。
- **参数**:
  - `joy_mode` (string, 默认: "d"): 手柄模式（'d' 或 'x'）。
  - `throttle_scale` (double, 默认: 0.5): 油门比例。
  - `servo_scale` (double, 默认: 1.0): 转向比例。

## 使用方法

### 1. 键盘控制 (Keyboard)

由于 `ros2 launch` 可能会拦截终端输入，**强烈建议**在独立的终端窗口中运行键盘节点：

**步骤 A: 启动转换节点（可选，如果您需要将 Twist 转为 Ackermann）**
```bash
ros2 launch tianracer_teleop keyboard_teleop.launch.py
```

**步骤 B: 在新终端运行键盘交互**
```bash
ros2 run tianracer_teleop teleop_twist_keyboard.py --ros-args -r __ns:=/tianracer
```

### 2. 手柄控制 (Joystick)

将手柄插入电脑/车载电脑后运行：

```bash
ros2 launch tianracer_teleop joystick_teleop.launch.py
```

- **默认设置**: 
  - 通过 `TIANRACER_JOY_MODE` 环境变设置模式。
  - 通过 `TIANRACER_JOY_DEV` 设置设备路径（默认 `/dev/tianbot_joystick`）。
- **操作**: 
  - 按住 **L1 (Button 4)** 配合左摇杆进行油门控制。
  - 按住 **R1 (Button 5)** 配合左摇杆进行转向控制。
