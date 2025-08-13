#! /usr/bin/env python3
# LastEditors: sujit-168 su2054552689@gmail.com
# LastEditTime: 2024-03-22 10:23:42

"""
the brief: subscribe to tf to calculate the l2 norm of the current pose and the target point and control early publish with the distance tolerance value
problem 1: direct subscriptions to odom topics are not posted frequently enough
"""

import os, math
import rospy, rospkg
import tf2_ros
import actionlib
import waypoint_race.utils as utils
import tf2_msgs.msg as tf2_msgs
import move_base_msgs.msg as move_base_msgs
import visualization_msgs.msg as viz_msgs

world = os.getenv("TIANRACER_WORLD", "tianracer_racetrack")
robot_name = os.getenv("TIANRACER_NAME", "tianracer")

class RaceStateMachine(object):
    def __init__(self, filename, repeat=True):
        """
        init RaceStateMachine
        filename: path to your yaml file
        reapeat: determine whether to visit waypoints repeatly(usually True in 110)
        """
        self._waypoints = utils.get_waypoints(filename) # gets the value of a series of target points

        action_name = 'move_base'
        self._ac_move_base = actionlib.SimpleActionClient(action_name, move_base_msgs.MoveBaseAction) # simple action client
        rospy.loginfo('Wait for %s server' % action_name)
        self._ac_move_base.wait_for_server
        self._counter = 0
        self._repeat = repeat
        self._early_pub = False
        self._tolerance_length = 0

        # publish the first goal
        rospy.sleep(0.2)     # wait for move_base to be ready, it is necessary and very important
        self._current_goal = utils.create_move_base_goal(self._waypoints[self._counter])
        self._ac_move_base.send_goal(self._current_goal)
        self._counter += 1
        
        # initial tf buffer
        self._buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._buffer)

        # the following is to display the target point
        self._pub_viz_marker = rospy.Publisher('viz_waypoints', viz_msgs.MarkerArray, queue_size=1, latch=True)
        self._viz_markers = utils.create_viz_markers(self._waypoints)

    def _rest_of_path(self):
        """
        count how many length of left
        """
        rospy.sleep(0.1)   # wait for 0.1s, ensure tf buffer is updated

        trans = self._buffer.lookup_transform(robot_name + "/base_footprint", robot_name + "/odom", rospy.Time(0))
        current_goal = self._current_goal.target_pose.pose.position
        rospy.loginfo("current goal x: %s, y: %s", current_goal.x, current_goal.y)

        length = math.sqrt(math.pow((trans.transform.translation.x - current_goal.x), 2) + math.pow((trans.transform.translation.y - current_goal.y), 2))
        
        self._tolerance_length = 3    # when controlling how far the current position is from the target point the next target point is sent
        if length < self._tolerance_length:
            self._early_pub = True
        else:
            self._early_pub = False
            rospy.loginfo("the distance between current goal and pose: %d", length)
    
    def move_to_next(self):
        pos = self._get_next_destination()
        if not pos:
            rospy.loginfo("Finishing Race")
            return True
        # convert the target point information read from the file into the format of the goal of move base
        goal = utils.create_move_base_goal(pos)
        self._current_goal = utils.create_move_base_goal(pos)
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
            self._rest_of_path()
            if self._early_pub:
                self.move_to_next()
                self._early_pub = False
            else:
                rospy.sleep(1.0)   # controls the length of time the vehicle can continue to drive after sending the target point

if __name__ == '__main__':
    rospy.init_node('race')
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