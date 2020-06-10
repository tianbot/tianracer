#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy


class TianracerJoyTeleop(object):
    """Tianracer joy teleop node. rewrite from cpp turtlebot logitech teleop"""

    def __init__(self):
        rospy.loginfo("Tianracer JoyTeleop Initializing...")
        self._ackermann = AckermannDrive()
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        self._zero_ackermann = self._ackermann
        self._deadman_pressed = False
        self._zero_ackermann_published = False

        self._ackermann_cmd_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self._timer = rospy.Timer(rospy.Duration(0.05), self.joystick_controller)
        self._axis_throttle = 1

        _joy_mode = rospy.get_param("~joy_mode", "D").lower()
        if _joy_mode == "d":
            self._axis_servo = 2
        if _joy_mode == "x":
            self._axis_servo = 3

        self._throttle_scale = rospy.get_param("~throttle_scale", 0.5)
        self._servo_scale = rospy.get_param("~servo_scale", 1)

    def joy_callback(self, joy):
        # reset the speed every cycle.
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        if joy.buttons[4] == 1:
            self._ackermann.speed = joy.axes[self._axis_throttle] * self._throttle_scale * 3

        if joy.buttons[5] == 1:
            self._ackermann.steering_angle = joy.axes[self._axis_servo] * self._servo_scale * 30/180*3.1415926535

        self._deadman_pressed = joy.buttons[4] or joy.buttons[5]

    def joystick_controller(self, *args):
        if self._deadman_pressed:
            self._ackermann_cmd_pub.publish(self._ackermann)
            self._zero_ackermann_published = False
        elif not self._zero_ackermann_published:
            self._ackermann_cmd_pub.publish(self._zero_ackermann)
            self._zero_ackermann_published = True


if __name__ == '__main__':
    try:
        rospy.init_node('tianracer_joy_teleop', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("Init node tianracer_joy_teleop failed, please check your main ROS core func.")
    else:
        TianracerJoyTeleop()
        rospy.spin()
