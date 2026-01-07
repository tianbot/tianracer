#!/usr/bin/env python3
# Author: tianbot (Modified for 50Hz upsampling)
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class AckermannConverter:
    def __init__(self):
        rospy.init_node('ackermann_stamped_converter', anonymous=True)
        
        # 参数获取
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', 'ackermann_cmd')
        ackermann_stamped_cmd_topic = rospy.get_param('~ackermann_stamped_cmd_topic', 'ackermann_cmd_stamped')
        
        # 初始化为空，用于判断是否收到了第一条指令
        self.latest_drive_msg = None
        
        # 发布者
        self.drive_pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
        
        # 订阅者
        rospy.Subscriber(ackermann_stamped_cmd_topic, AckermannDriveStamped, self.convert_callback, queue_size=1)
        
        # --- 设定为 50Hz --- ，与下位机超时频率一致
        # 频率 f = 50, 周期 T = 1/50 = 0.02s
        self.timer = rospy.Timer(rospy.Duration(1.0/50.0), self.timer_callback)

    def convert_callback(self, msg):
        """将收到的 Stamped 消息解包并存入缓存"""
        if self.latest_drive_msg is None:
            self.latest_drive_msg = AckermannDrive()
            
        self.latest_drive_msg.speed = msg.drive.speed
        self.latest_drive_msg.steering_angle = msg.drive.steering_angle
        self.latest_drive_msg.acceleration = msg.drive.acceleration
        self.latest_drive_msg.steering_angle_velocity = msg.drive.steering_angle_velocity
        self.latest_drive_msg.jerk = msg.drive.jerk

    def timer_callback(self, event):
        """以 50Hz 频率持续发布最新缓存的数据"""
        if self.latest_drive_msg is not None:
            self.drive_pub.publish(self.latest_drive_msg)

if __name__ == '__main__':
    try:
        converter = AckermannConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass