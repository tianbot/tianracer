#! /usr/bin/env python3
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

"""
the brief: use move base navfn ros plan to control early publishing by subscribing to the length rate of the number of waypoints for /global_plan
problem 1: the move base navfn ros plan topic is not posted frequently enough
"""

import os
import rospy, rospkg
import actionlib 
import waypoint_race.utils as utils
import nav_msgs.msg as nav_msgs
import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs
import subprocess

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # gets the value of a series of target points

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # create one SimpleActionClient
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._early_pub = False
        self._len_path_list = []
        self._tolerance_length = 0
        self._min_tolerance_length = 300

        # pub first goal
        rospy.sleep(3)     # wait for move_base to be ready, it is necessary and very important
        self._current_goal = utils.create_move_base_goal(self._waypoints[self._counter])
        self._ac_move_base.send_goal(self._current_goal)
        self._counter += 1
        rospy.loginfo("the status of move_base: %s", self._ac_move_base.get_state())

        # clear_costmap
        self._clear_costmap_command  = (f"rosservice call move_base/clear_costmaps")
        
        # the following is to display the target point
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)
        
        # start subscribing to the global path
        # self._rest_of_path = rospy.Subscriber("move_base/NavfnROS/plan", nav_msgs.Path, self._rest_of_path_callback, queue_size=1)                   # bug: the release frequency is not enough only about 2hz and the move base parameters can be adjusted to improve
        self._rest_of_path = rospy.Subscriber("move_base/TebLocalPlannerROS/global_plan", nav_msgs.Path, self._rest_of_path_callback, queue_size=1)    # the release frequency is around 10hz

    def _rest_of_path_callback(self, data):
        """
        count how many waypoints left
        """

        rospy.loginfo("the status of move_base: %s", self._ac_move_base.get_state())
        length = len(data.poses)
       
        if not self._len_path_list:
            rospy.sleep(0.2)   # wait for the new global planner path to be ready
            self._len_path_list.append(length)
            rospy.loginfo("len_path_list: %s", self._len_path_list)
            self._tolerance_length = max(self._len_path_list) / 2.5    # controls how many transit points are left in the currently planned route and sends the next destination point
            
            # limit of value
            if self._tolerance_length < self._min_tolerance_length:
                self._tolerance_length = self._min_tolerance_length      # avoid the value is too small
            rospy.loginfo("the tolerance: %d", self._tolerance_length)
        else:
            pass
        if length < self._tolerance_length:
            self._early_pub = True
        else:
            self._early_pub = False
            rospy.loginfo("the length of current global waypoints: %d", length)
    
    def move_to_next(self):
        pos = self._get_next_destination()

        if not pos:
            rospy.loginfo("Finishing Race")
            return True
        # convert the target point information read from the file into the format of the goal of move base
        goal = utils.create_move_base_goal(pos)
        rospy.loginfo("Move to %s" % pos['name'])

        # here s a very simple send goal
        self._ac_move_base.send_goal(goal)

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
        self._pub_viz_marker.publish(self._viz_markers)
        
        while not rospy.is_shutdown() and self._repeat:
            if self._early_pub:
                self._len_path_list = []
                subprocess.Popen(self._clear_costmap_command, shell=True)
                self.move_to_next()
                self._early_pub = False
            else:
                rospy.sleep(1.0)   # controls the length of time the vehicle can continue to drive after sending the target point

if __name__ == '__main__':
    rospy.init_node('multi_goals')
    package_name = "tianracer_gazebo"

    # Get the package path
    try:
        pkg_path = rospkg.RosPack().get_path(package_name)

        # Construct the path to scripts directory
        filename= os.path.join(pkg_path, f"scripts/waypoint_race/{world}_points.yaml")
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