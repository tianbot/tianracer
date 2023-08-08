#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import Joy

msg = """
Control the Tianracer through Gamepad
---------------------------
Moving around:
Hold LB or RB button to enable manual control
Press X button to switch from Real to Sim

LEFT HAND AXIS:             RIGHT HAND AXIS:

        +throttle                 
            ↑                        ↑
         ←     →        +steering ←     → -steering
            ↓                        ↓
        -throttle                 
  
Buttons:
X: Switch from Real to Sim

CTRL-C to quit
"""


class TianracerJoyTeleop(object):
    """Tianracer joy teleop node. rewrite from cpp turtlebot logitech teleop"""

    def __init__(self):
        print(msg)
        # for real car we publish AckermannDrive
        self._ackermann = AckermannDrive()
        # for sim car we publish AckermannDriveStamped
        self._ackermann_stamped = AckermannDriveStamped()
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        self._zero_ackermann = AckermannDrive()
        self._deadman_pressed = False
        self._sim_mode = False
        self._zero_ackermann_published = False

        self._ackermann_cmd_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=5)
        self._ackermann_cmd_stamped_pub = rospy.Publisher('tianracer/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=5)
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self._timer = rospy.Timer(rospy.Duration(0.05), self.joystick_controller)
        self._axis_throttle = 1

        self._joy_mode = rospy.get_param("~joy_mode", "D").lower()        
        if self._joy_mode == "d":
            self._axis_throttle = 1
            self._axis_steering = 2
            self._btn_switch = 0

        self._throttle_max = rospy.get_param("~throttle_max", 0.5)
        self._servo_max = rospy.get_param("~servo_max", 1)
        rospy.loginfo("Publish to tianracer/ackermann_cmd")

    def joy_callback(self, joy):
        # reset the speed every cycle.
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        
        if len(joy.axes) == 6:
            if not self._joy_mode == "d":
                self._joy_mode = "d"
                rospy.loginfo("Joypad is set to mode 'd'")
                self._axis_throttle = 1
                self._axis_steering = 2
                self._btn_switch = 0
        elif len(joy.axes) == 8:
            if not self._joy_mode == "x":
                self._joy_mode = "x"
                rospy.loginfo("Joypad is set to mode 'x'")
                self._axis_throttle = 1
                self._axis_steering = 3
                self._btn_switch = 2

        # LB and RB are always buttons 4 and 5
        self._deadman_pressed = joy.buttons[4] or joy.buttons[5]
        
        self._ackermann.speed = joy.axes[self._axis_throttle] * self._throttle_max
        self._ackermann.steering_angle = joy.axes[self._axis_steering] * self._servo_max
        if joy.buttons[self._btn_switch] == 1:
            self._sim_mode = not self._sim_mode
            if self._sim_mode:
                rospy.loginfo("Publish to tianracer/ackermann_cmd_stamped")
            else:
                rospy.loginfo("Publish to tianracer/ackermann_cmd")

    def joystick_controller(self, *args):
        if self._deadman_pressed:
            if self._sim_mode:
                self._ackermann_stamped.drive = self._ackermann
                self._ackermann_cmd_stamped_pub.publish(self._ackermann_stamped)
            else:
                self._ackermann_cmd_pub.publish(self._ackermann)
            self._zero_ackermann_published = False
        elif not self._zero_ackermann_published:
            if self._sim_mode:
                self._ackermann_stamped.drive = self._zero_ackermann
                self._ackermann_cmd_stamped_pub.publish(self._ackermann_stamped)
            else:
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
