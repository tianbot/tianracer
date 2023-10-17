#! /usr/bin/env python
# source code was from https://www.guyuehome.com/35146
# modified by Lr
import random
import rospy
import actionlib # 引用actionlib库
import waypoint_touring.utils as utils # 代码附在后面

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

class TourMachine(object):

    def __init__(self, filename, random_visits=False, repeat=True):
        self._waypoints = utils.get_waypoints(filename) # 获取一系列目标点的值

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # 创建一个SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = True
        self._random_visits = random_visits

        if self._random_visits:
            random.shuffle(self._waypoints)
        # 以下为了显示目标点：
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def move_to_next(self):
        p = self._get_next_destination()

        if not p:
            rospy.loginfo("Finishing Tour")
            return True
        # 把文件读取的目标点信息转换成move_base的goal的格式：
        goal = utils.create_move_base_goal(p)
        rospy.loginfo("Move to %s" % p['name'])
        # 这里也是一句很简单的send_goal:
        self._ac_move_base.send_goal(goal)
        self._ac_move_base.wait_for_result()
        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)

        return False

    def _get_next_destination(self):
        """
        根据是否循环，是否随机访问等判断，决定下一个目标点是哪个
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
                if self._random_visits:
                    random.shuffle(self._waypoints)
            else:
                next_destination = None
        next_destination = self._waypoints[self._counter]
        self._counter = self._counter + 1
        return next_destination

    def spin(self):
        rospy.sleep(1.0)
        self._pub_viz_marker.publish(self._viz_markers)
        finished = False
        while not rospy.is_shutdown() and not finished:
            finished = self.move_to_next()
            rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('tour')
    # 在这里直接限定你的路径名称
    filename = "/home/tianbot/tianbot_ws/src/tianracer/tianracer_gazebo/scripts/waypoint_touring/points.yaml"

    random_visits = rospy.get_param('~random', False)
    repeat = rospy.get_param('~repeat', False)

    m = TourMachine(filename, random_visits, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Bye Bye')