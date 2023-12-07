#! /usr/bin/env python
# @Time: 2023/10/20 17:04:32
# @Author: Jeff Wang(Lr_2002)
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64MultiArray

plt.ion()
xx = [i - 60 for i in range(112)]
yy = [1.1 for i in range(112)]
cnt = 0
def plot_callback(data):
    global cnt
    cnt +=1 
    if cnt <5 :
        print(cnt)
        return
    dis_list = data.data
    # print(dis_list)
    plt.clf()
    
    plt.plot(xx, dis_list)
    plt.plot(xx, yy)
    plt.pause(0.001)
    plt.ioff()
    cnt =0

if __name__ == '__main__':
    try:
        rospy.init_node("plot_image")
        image_sub = rospy.Subscriber("/image", Float64MultiArray, plot_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass