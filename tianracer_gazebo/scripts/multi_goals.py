#! /usr/bin/env python
# @Source: https://www.guyuehome.com/35146
# @Time: 2023/10/20 17:02:46
# @Author: Jeff Wang(Lr_2002)
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

import os
import rospy, rospkg
import actionlib # 引用 actionlib 库
import waypoint_race.utils as utils

import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs
    
class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # 获取一系列目标点的值

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # 创建一个 SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat


        # 以下为了显示目标点：
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def move_to_next(self):
        pos = self._get_next_destination()

        if not pos:
            rospy.loginfo("Finishing Race")
            return True
        # 把文件读取的目标点信息转换成 move_base 的 goal 的格式：
        goal = utils.create_move_base_goal(pos)
        rospy.loginfo("Move to %s" % pos['name'])
        # 这里也是一句很简单的 send_goal:
        self._ac_move_base.send_goal(goal)
        self._ac_move_base.wait_for_result()
        result = self._ac_move_base.get_result()
        rospy.loginfo("Result : %s" % result)

        return False

    def _get_next_destination(self):
        """
        determine what's the next goal point according to repeat 
        """
        if self._counter == len(self._waypoints):
            if self._repeat:
                self._counter = 0
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
    rospy.init_node('race')
    
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, "scripts/waypoint_race/points.yaml")
        print(f"yaml: {filename}")
    except rospkg.ResourceNotFound:
        rospy.logerr("Package '%s' not found" % package_name)
        exit(1)

    filename = rospy.get_param("~filename",filename)
    repeat = rospy.get_param('~repeat', True)

    m = RaceStateMachine(filename, repeat)
    rospy.loginfo('Initialized')
    m.spin()
    rospy.loginfo('Finished')