#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    if settings is None:
        return sys.stdin.read(1)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def main():
    settings = None
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    speed = node.declare_parameter("speed", 0.5).value
    turn = node.declare_parameter("turn", 1.0).value
    speed_limit = node.declare_parameter("speed_limit", 1000.0).value
    turn_limit = node.declare_parameter("turn_limit", 1000.0).value
    key_timeout = node.declare_parameter("key_timeout", 0.5).value
    stamped = node.declare_parameter("stamped", False).value
    frame_id = node.declare_parameter("frame_id", '').value

    if stamped:
        pub = node.create_publisher(TwistStamped, 'cmd_vel', 10)
    else:
        pub = node.create_publisher(Twist, 'cmd_vel', 10)

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0

    try:
        if settings is None:
            print("Warning: stdin is not a TTY. Keyboard teleop will not work if launched via ros2 launch without a proper terminal.")
            print("Please run this node using 'ros2 run tianracer_teleop teleop_twist_keyboard.py' in a separate terminal.")
            
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if stamped:
                msg_out = TwistStamped()
                msg_out.header.stamp = node.get_clock().now().to_msg()
                msg_out.header.frame_id = frame_id
                msg_out.twist.linear.x = x * speed
                msg_out.twist.linear.y = y * speed
                msg_out.twist.linear.z = z * speed
                msg_out.twist.angular.z = th * turn
            else:
                msg_out = Twist()
                msg_out.linear.x = x * speed
                msg_out.linear.y = y * speed
                msg_out.linear.z = z * speed
                msg_out.angular.z = th * turn

            pub.publish(msg_out)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            msg_out = TwistStamped()
            msg_out.header.stamp = node.get_clock().now().to_msg()
            msg_out.header.frame_id = frame_id
        else:
            msg_out = Twist()
        pub.publish(msg_out)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
