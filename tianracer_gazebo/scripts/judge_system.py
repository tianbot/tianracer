#! /usr/bin/env python
# @Time: 2023/10/20  17:01:19
# @Author: Jeff Wang(Lr_2002)


import tkinter as tk
import rospy 
from std_msgs.msg import String
score = 60
from datetime import datetime


import random
import threading
random.seed(1)

"""
todo:
    update the code backend

"""

class Judger:
    def __init__(self):
        self.root_window = tk.Tk()

        self.scores = 0
        self.times = 0
        self.infos = 0

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

        self.start_button = tk.Button(self.root_window, text="启动",command=start)
        self.start_button.place(x=80, y= 500, width=300,height=60)


        self.info_item = tk.Text(self.root_window, width=40, height=400)
        self.info_item.pack(side="right")

        self.update_info_item("正在启动...")
        self.update_info_item("成功启动程序，开始测试")
        self.update_info_item("通过A点+3分")
        # update_info(self.info_item, "hello\n")   

    def start(self):
        self.update_info()
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()
        self.root_window.mainloop()
    
    def update_info(self):
        info = 132.123
        self.score_display.configure(text=str(int(info)))
        self.time_count.configure(text="10:02:32")
        self.root_window.after(100, self.update_info)

    def update_info_item(self, info):
        now = datetime.now()

        current_time = '[' + now.strftime("%H:%M:%S")+']  '
        self.info_item.insert(tk.INSERT, current_time+info+'\n')
        return True

def start():
    return True

# def update_info(info_box, info:str):
#     info_box.insert(tk.INSERT, info)
#     return True


def update_callback(info):
    print("recv info", info.data)
    judger.update_info_item(info.data)
    

if __name__=="__main__":
    judger = Judger()

    rospy.init_node("judger")
    position_sub = rospy.Subscriber("/position", String, callback=update_callback)
    # rospy.spin()
    judger.start()


    