"""Offline regression tests: real ROS messages, mocked ROS transport."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('waypoint_laps', SCRIPTS / 'waypoint_laps.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WaypointLapsTests(unittest.TestCase):
    def setUp(self):
        self.ros = patch.object(module, 'rospy').start()
        self.ros.is_shutdown.return_value = False
        patch.object(module.utils.rospy.Time, 'now', return_value=module.utils.rospy.Time(0)).start()
        self.ros.Publisher.side_effect = lambda *args, **kwargs: Mock()
        patch.object(module.time, 'sleep').start()
        self.client = patch.object(module.actionlib, 'SimpleActionClient').start().return_value
        self.client.get_state.return_value = GoalStatus.SUCCEEDED
        self.addCleanup(patch.stopall)
        self.filename = str(SCRIPTS / 'waypoint_race' / 'tianracer_racetrack_points.yaml')

    def race(self, **kwargs):
        return module.WaypointLaps(self.filename, **kwargs)

    def test_five_closed_laps_and_zero_stop(self):
        race = self.race()
        self.assertTrue(race.spin())
        self.assertEqual(race.completed_laps, 5)
        goals = [call.args[0].target_pose.pose.position for call in self.client.send_goal.call_args_list]
        positions = [(w['pose']['position']['x'], w['pose']['position']['y']) for w in race.waypoints]
        self.assertEqual([(p.x, p.y) for p in goals], [positions[0]] + (positions[1:] + [positions[0]]) * 5)
        callbacks = [call.kwargs['feedback_cb'] for call in self.client.send_goal.call_args_list]
        self.assertIsNone(callbacks[0])
        self.assertIsNone(callbacks[-1])
        self.assertTrue(all(cb is not None for cb in callbacks[1:-1]))
        self.client.wait_for_server.assert_called_once_with()
        self.assertEqual(race._stop_pub.publish.call_count, 3)
        for call in race._stop_pub.publish.call_args_list:
            self.assertEqual(call.args[0].linear.x, 0)
            self.assertEqual(call.args[0].angular.z, 0)

    def test_one_lap(self):
        race = self.race(laps=1)
        self.assertTrue(race.spin())
        self.assertEqual(race.completed_laps, 1)
        self.assertEqual(self.client.send_goal.call_count, len(race.waypoints) + 1)

    def test_failed_final_arrival_does_not_count_fifth_lap(self):
        race = self.race()
        self.client.get_state.side_effect = [GoalStatus.SUCCEEDED] * 20 + [GoalStatus.ABORTED] * 2
        self.assertFalse(race.spin())
        self.assertEqual(race.completed_laps, 4)
        self.assertEqual(self.client.send_goal.call_count, 21)

    def test_zero_distance_disables_early_switch(self):
        race = self.race(laps=1, switch_distance=0.0)
        self.assertTrue(race.spin())
        self.assertTrue(all(call.kwargs['feedback_cb'] is None
                            for call in self.client.send_goal.call_args_list))

    def test_navigation_failures_do_not_count_laps(self):
        for status in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
            with self.subTest(status=status):
                self.client.send_goal.reset_mock()
                self.client.get_state.side_effect = [GoalStatus.SUCCEEDED, status, status]
                race = self.race()
                self.assertFalse(race.spin())
                self.assertEqual(race.completed_laps, 0)
                self.assertEqual(self.client.send_goal.call_count, 2)
                self.client.cancel_goal.assert_called()
        self.client.get_state.side_effect = None

    def test_timeout_cancels_without_advancing(self):
        race = self.race()
        self.client.get_state.side_effect = [GoalStatus.ACTIVE, GoalStatus.PREEMPTED]
        with patch.object(module.time, 'monotonic', side_effect=[0, 121, 121]):
            self.assertFalse(race.spin())
        self.assertEqual(race.completed_laps, 0)
        self.client.cancel_goal.assert_called_once()
        self.assertEqual(self.client.send_goal.call_count, 1)

    def deliver_feedback(self, frame='map', offset=0.2):
        def send(goal, feedback_cb=None):
            feedback = MoveBaseFeedback()
            feedback.base_position.header.frame_id = frame
            feedback.base_position.pose.position.x = goal.target_pose.pose.position.x + offset
            feedback.base_position.pose.position.y = goal.target_pose.pose.position.y
            if feedback_cb:
                feedback_cb(feedback)
        self.client.send_goal.side_effect = send

    def test_distance_feedback_allows_early_switch(self):
        race = self.race()
        self.deliver_feedback()
        self.client.get_state.return_value = GoalStatus.ACTIVE
        self.assertTrue(race._visit(1, allow_early=True))

    def test_wrong_frame_or_distant_feedback_cannot_advance(self):
        for frame, offset in (('odom', 0.2), ('map', 3.0), ('map', float('nan'))):
            with self.subTest(frame=frame, offset=offset):
                race = self.race()
                self.deliver_feedback(frame, offset)
                self.client.get_state.return_value = GoalStatus.ACTIVE
                with patch.object(module.time, 'monotonic', side_effect=[0, 121]):
                    self.assertFalse(race._visit(1, allow_early=True))

    def test_last_goal_requires_success(self):
        race = self.race()
        self.deliver_feedback()
        self.client.get_state.return_value = GoalStatus.ACTIVE
        with patch.object(module.time, 'monotonic', side_effect=[0, 121]):
            self.assertFalse(race._visit(0, allow_early=False))
        self.assertIsNone(self.client.send_goal.call_args.kwargs['feedback_cb'])

    def test_invalid_parameters_and_empty_route(self):
        for kwargs in ({'laps': 0}, {'laps': True}, {'laps': 1.5},
                       {'switch_distance': -1}, {'switch_distance': float('nan')},
                       {'goal_timeout': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.race(**kwargs)
        with patch.object(module.utils, 'get_waypoints', return_value=[]):
            with self.assertRaises(ValueError):
                self.race()
        self.client.send_goal.assert_not_called()

    def test_shutdown_does_not_count_a_lap_and_stop_is_idempotent(self):
        race = self.race()
        self.ros.is_shutdown.return_value = True
        self.client.get_state.return_value = GoalStatus.PREEMPTED
        self.assertFalse(race.spin())
        self.assertEqual(race.completed_laps, 0)
        self.client.cancel_goal.assert_called_once()
        race.stop()
        self.assertEqual(race._stop_pub.publish.call_count, 3)


if __name__ == '__main__':
    unittest.main()
