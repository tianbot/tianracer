#!/usr/bin/env python3
"""Run a finite number of closed waypoint laps using ROS 1 move_base."""

import math
import os
import threading
import time

import actionlib
import rospkg
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from visualization_msgs.msg import MarkerArray

from waypoint_race import utils


class WaypointLaps:
    def __init__(self, filename, laps=5, switch_distance=0.5, goal_timeout=120.0):
        if isinstance(laps, bool) or not isinstance(laps, int) or laps < 1:
            raise ValueError('laps must be a positive integer')
        if not math.isfinite(switch_distance) or switch_distance < 0:
            raise ValueError('switch_distance must be finite and nonnegative')
        if not math.isfinite(goal_timeout) or goal_timeout <= 0:
            raise ValueError('goal_timeout must be finite and positive')
        self.waypoints = utils.get_waypoints(filename)
        if len(self.waypoints) < 2:
            raise ValueError('A closed lap needs at least two waypoints')
        for waypoint in self.waypoints:
            if not waypoint['frame_id']:
                raise ValueError('Every waypoint needs a frame_id')
            position = waypoint['pose']['position']
            if not all(math.isfinite(position[axis]) for axis in ('x', 'y', 'z')):
                raise ValueError('Waypoint positions must be finite')
        self.laps = laps
        self.switch_distance = switch_distance
        self.goal_timeout = goal_timeout
        self.completed_laps = 0
        self._goal_active = False
        self._stopped = False
        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._stop_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self._marker_pub = rospy.Publisher('viz_waypoints', MarkerArray,
                                          queue_size=1, latch=True)
        rospy.on_shutdown(self.stop)
        rospy.loginfo('Waiting for %s', rospy.resolve_name('move_base'))
        self._client.wait_for_server()

    def _visit(self, index, allow_early):
        allow_early = allow_early and self.switch_distance > 0
        goal = utils.create_move_base_goal(self.waypoints[index])
        near_goal = threading.Event()

        def feedback_callback(feedback):
            # A per-goal event keeps late feedback from advancing another goal.
            pose = feedback.base_position
            if pose.header.frame_id.lstrip('/') != goal.target_pose.header.frame_id.lstrip('/'):
                rospy.logwarn_throttle(5.0, 'Feedback and goal frames differ; waiting for SUCCEEDED')
                return
            current = pose.pose.position
            target = goal.target_pose.pose.position
            if math.hypot(current.x - target.x, current.y - target.y) <= self.switch_distance:
                near_goal.set()

        rospy.loginfo('Lap %d/%d, waypoint %d/%d: %s',
                      self.completed_laps + 1, self.laps, index + 1,
                      len(self.waypoints), self.waypoints[index]['name'])
        self._goal_active = True
        self._client.send_goal(goal, feedback_cb=feedback_callback if allow_early else None)
        deadline = time.monotonic() + self.goal_timeout
        while not rospy.is_shutdown():
            state = self._client.get_state()
            if state == GoalStatus.SUCCEEDED:
                self._goal_active = False
                return True
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                rospy.logerr('Waypoint %d failed (state %d): %s', index + 1,
                             state, self._client.get_goal_status_text())
                return False
            if state == GoalStatus.ACTIVE and allow_early and near_goal.is_set():
                return True
            if time.monotonic() >= deadline:
                rospy.logerr('Waypoint %d timed out after %.1f wall-clock seconds',
                             index + 1, self.goal_timeout)
                return False
            # Wall time allows shutdown/timeout while Gazebo is paused.
            time.sleep(0.05)
        return False

    def spin(self):
        try:
            self._marker_pub.publish(utils.create_viz_markers(self.waypoints))
            # Establish the lap origin. Merely sending the first goal is not a lap.
            if not self._visit(0, allow_early=False):
                return False
            for lap in range(self.laps):
                for index in range(1, len(self.waypoints)):
                    if not self._visit(index, allow_early=True):
                        return False
                # Close the loop explicitly; the final arrival must be SUCCEEDED.
                if not self._visit(0, allow_early=lap < self.laps - 1):
                    return False
                self.completed_laps += 1
                rospy.loginfo('Completed lap %d/%d', self.completed_laps, self.laps)
            return True
        finally:
            self.stop()

    def stop(self):
        if self._stopped:
            return
        self._stopped = True
        if self._goal_active:
            # Cancel this client's goal, never goals belonging to other clients.
            self._client.cancel_goal()
            deadline = time.monotonic() + 2.0
            while self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                if time.monotonic() >= deadline:
                    rospy.logwarn('move_base cancellation acknowledgement timed out')
                    break
                time.sleep(0.05)
        # Gazebo nav_sim converts this Twist to a zero Ackermann command.
        for _ in range(3):
            self._stop_pub.publish(Twist())
            time.sleep(0.05)


def main():
    rospy.init_node('waypoint_laps')
    world = rospy.get_param('~world', os.getenv('TIANRACER_WORLD', 'tianracer_racetrack'))
    filename = rospy.get_param('~filename', '')
    if not filename:
        filename = os.path.join(rospkg.RosPack().get_path('tianracer_gazebo'),
                                'scripts', 'waypoint_race', world + '_points.yaml')
    race = WaypointLaps(filename, rospy.get_param('~laps', 5),
                        rospy.get_param('~switch_distance', 0.5),
                        rospy.get_param('~goal_timeout', 120.0))
    if race.spin():
        rospy.loginfo('Finished all %d laps', race.completed_laps)
        return 0
    rospy.logerr('Race stopped after %d/%d laps', race.completed_laps, race.laps)
    return 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
    except (ValueError, KeyError, OSError, rospkg.ResourceNotFound) as error:
        rospy.logerr('Cannot run waypoint laps: %s', error)
        raise SystemExit(1)
