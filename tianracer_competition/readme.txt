author: Wu GengQian
date: 2021-8-13

系统版本：Ubuntu 20.04.1 LTS
处理器：AMD® Ryzen 5 4500u with radeon graphics × 6
内存：15.1 GiB

算法概述：
https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html

跑封闭地图：
roslaunch tianracer_competition closed_map.launch speed_param:=3.5 P_param:=0.0
其中，speed_param为直线行驶速度，默认值：3.5
P_param为转弯减速比例，默认值：0.0
在本机测试结果：speed_param:=6 P_param:=3，无障碍单圈完成时间13秒左右。

跑开放地图：
roslaunch tianracer_competition open_map.launch speed_param:=3.5 P_param:=0.0
其中，speed_param为直线行驶速度，默认值：3.5
P_param为转弯加速比例，默认值：0.0
在本机测试结果：speed_param:=4.5 P_param:=1，无障碍单圈完成时间15秒左右。

tips：
1. 该算法可能在入弯或者出弯的时候由于障碍物的特殊位置导致失败。
2. 在测试过程中发现，simulator在不同电脑性能条件下效果不同，由于时间关系并未仔细验证。
3. 以上跑图参数仅在本机测试通过，仅供参考。
4. 若在ubuntu 18.04运行程序，python运行环境python3改为python。
