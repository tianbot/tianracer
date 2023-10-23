#! /usr/bin/env python
# @Time: 2023/10/20  17:01:19
# @Author: Jeff Wang(Lr_2002)


import tkinter as tk
import rospy 
from std_msgs.msg import String
import os
from datetime import datetime
import time
from utils.position_check import *
import subprocess  
import random
import threading
random.seed(1)

"""
todo 
    1. stop all thread when finish
    2. stop when no feedback 
    3. the 10 point will stll get score output


"""



map_list = [5, 5, 5, 5, 5, 5, 10, 10, 15]
"""
todo:
    update the code backend

"""

class Judger:
    def __init__(self):
        self.root_window = tk.Tk()

        self.scores = 0
        self.time_back = time.time()
        self.times = 0
        self.infos = 0
        self.pass_point = None
        self.start_flag= False

        self.root_window.geometry('800x600')
        self.root_window.title("Tianbot 官方监控系统")

        # self.timer = tk.Label(self.root_window,  text="时间:", anchor='w', fg="black",font=('Times', 20, 'bold italic'), width=30, height=5,padx=3, pady=3,borderwidth=3)
        # self.timer.place(x=100, y=100,width=140, height=100)

        self.time_count = tk.Label(self.root_window,text="30" + 's', anchor='w',  fg="red",font=('DSEG7Classic', 25, 'bold italic'), width=30, height=5,padx=3, pady=3,borderwidth=3)
        self.time_count.place(x=150, y=100, width=250, height=100)

        self.score_info_box = tk.Label(self.root_window, relief='sunken',text="分数:", fg="red", anchor='w', font=('Times', 20, 'bold italic'),width=30, height=5,padx=3, pady=3,borderwidth=3)
        self.score_info_box.place(x=100, y= 300, width=140, height=100)

        self.score_display = tk.Label(self.root_window, relief='sunken', text='6', anchor='w',  fg="black",font=('Times', 20, 'bold italic'), width=30, height=5,padx=3, pady=3,borderwidth=3)
        self.score_display.place(x=240, y =300, width=160, height=100)

        self.start_button = tk.Button(self.root_window, text="启动",command=self.start_judge)
        self.start_button.place(x=80, y= 500, width=300,height=60)

        self.info_item = tk.Text(self.root_window, width=40, height=400)
        self.info_item.pack(side="right")

        # self.update_info_item("成功启动程序，开始测试")
        # self.update_info_item("通过A点+3分")
        # update_info(self.info_item, "hello\n")   

    def cal_time_score(self):
        self.scores += round(35 * 30  / max(self.times, 30),3)
        return self.scores

    def run_command(self):
        os.system("rosrun tianracer_gazebo f1tenth_racer.py")

    def start_judge(self):
        if self.start_flag == True:
            return False
        self.command_thread = threading.Thread(target=self.run_command)
        self.command_thread.start()
        self.update_info_item('目标代码已启动')
        self.time_back = time.time()

        self.start_flag =True
    def start(self):

        self.update_info()
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()
        self.update_info_item("测评系统已加载完成")
        self.root_window.mainloop()
    
    def update_info(self):
        # info = 132.123
        self.score_display.configure(text=self.scores)
        now = datetime.now()
        if self.start_flag:
            self.times = round(time.time() - self.time_back, 3)
        else:
            self.times = 0.000
        self.time_count.configure(text=self.times)
        self.root_window.after(100, self.update_info)

    def update_info_item(self, info):
        if info is None:
            return False
        score = None
        if isinstance(info, tuple):
            point = info[1]
            info = info[0]
            if point >= 9:
                return True
            global map_list
            score = map_list[point]
            self.scores += score
            self.pass_point = point
        now = datetime.now()

        current_time = '[' + now.strftime("%H:%M:%S")+']  '
        if not score:
            self.info_item.insert(tk.INSERT, current_time+info+'\n')
        else:   
            self.info_item.insert(tk.INSERT, current_time+info +' +' +str(score) +'分\n')

        if self.pass_point is not None and self.pass_point == 8:
            self.start_flag = False
            self.cal_time_score()
            self.info_item.insert(tk.INSERT, current_time+info +' +' +str(score) +'分\n')

        return True



def start():
    return True

# def update_info(info_box, info:str):
#     info_box.insert(tk.INSERT, info)
#     return True


def update_callback(data):
    res = analysis(data)
    judger.update_info_item(res)
    

if __name__=="__main__":
    judger = Judger()

    rospy.init_node("judger")
    # position_sub = rospy.Subscriber("/position", String, callback=update_callback)
    # rospy.spin()
    position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback=update_callback)
    judger.start()


    