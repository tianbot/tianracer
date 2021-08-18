author: Wu GengQian
date: 2021-8-13

System: Ubuntu 20.04.1 LTS
CPU: AMD® Ryzen 5 4500u with radeon graphics × 6
RAM: 15.1 GiB

Overview: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html

Run in a closed map：
roslaunch tianracer_competition closed_map.launch speed_param:=3.5 P_param:=0.0
speed_ param is the straight-line driving speed. Default: 3.5
P_ param is the turning deceleration ratio. Default: 0.0
Local test result: speed_ param:=6, P_ Param:=3, barrier free single lap completion time is about 13 seconds.

Run in an open map：
roslaunch tianracer_competition open_map.launch speed_param:=3.5 P_param:=0.0
speed_ param is the straight-line driving speed. Default: 3.5
P_ param is the turn acceleration ratio. Default: 0.0
Local test result: speed_ param:=4.5, P_ Param:=1, barrier free single lap completion time is about 15 seconds.

tips：
1. The algorithm may fail due to the special position of the obstacle when entering or leaving the corner.
2. During the test, it was found that the simulator had different effects under different computer performance conditions, but it was not carefully verified.
3. The above parameters only pass the local test and are for reference only.
4. To run the program on Ubuntu 20.04, execute sudo apt install python-is-python 3 on the terminal.
