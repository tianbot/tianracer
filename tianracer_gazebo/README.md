# Gazebo 多航点跑圈（ROS1-only）

先启动仿真、定位及 TEB 导航，再在另一个终端启动跑圈：

```bash
source /opt/ros/noetic/setup.bash
source /home/tianbot/tianbot_ws/devel/setup.bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch world:=tianracer_racetrack
```

```bash
source /opt/ros/noetic/setup.bash
source /home/tianbot/tianbot_ws/devel/setup.bash
roslaunch tianracer_gazebo waypoint_laps.launch robot_name:=tianracer laps:=5
```

运行前退出旧的多航点节点，避免多个客户端替换彼此的导航目标。
自定义地图时，两条命令传入相同的 `world`；自定义航点可传 `filename:=/绝对路径/points.yaml`。

`scripts/waypoint_laps.py` 替代旧的 `multi_goals_rc3.py`，无引用且存在缺陷的 rc1、rc2 已删除。
原始 `multi_goals.py` 保留，作为另一种逐点导航实现；本 launch 使用新脚本。

- 默认 `laps:=5`。先导航到 YAML 首点建立起点，再依次走其余点并回到首点，计为一圈。
- 中间目标通过当前 action 的 `base_position` 反馈判断距离；`switch_distance:=0.5` 的单位是米。
  此范围内可提前发下一目标，不要求中间点满足朝向容差；设置为 `0.0` 可关闭提前换点。
- 初始首点和第五圈结束的首点必须得到 `SUCCEEDED`，不使用提前换点。
- 帧不同的反馈不做距离判断，等待 action 成功。失败、外部抢占或超时立即结束，不跳过失败航点计圈。
- `goal_timeout:=120.0` 是每个目标的墙钟秒数，Gazebo 暂停也会计时。
- 完成、失败和 Ctrl+C 均取消仍在执行的当前目标，并发布零 `cmd_vel`。
  仿真中的 `nav_sim` 将其转换为零速 Ackermann 命令。

航点需要按赛道行驶顺序排列，并能引导规划器沿完整赛道行驶。
程序的“圈”是闭合航点序列，不能保证与 `tianracer_sim_referee` 的官方过线/检查点判圈一致。
默认点集只有四点，若规划器抄近路或方向不合理，需要在 RViz 检查路线并重新录点。
`SUCCEEDED` 仍要求末点朝向符合 TEB 的 `yaw_goal_tolerance`；车到点后难以完成时应检查所录朝向。

旧 rc3 的主要缺陷：没有有限圈数；使用 `len(Path.poses)` 和至少 300 点阈值换点，
点数不等于米数，空路径也会触发换点；旧路径反馈可能触发新目标切换；未处理成功/失败状态；
`repeat=False` 不运行主循环且末尾索引可能越界；每次换点异步调用清图服务，服务名还依赖环境变量。
新实现不再按路径点数换点，也不再每次换点清空代价地图。

离线回归检查（不会连接 ROS master 或发送车辆命令）：

```bash
source /opt/ros/noetic/setup.bash
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_waypoint_laps.py' -v
```

ROS 1 Noetic 已结束官方支持；这里维护现有仿真链路。
