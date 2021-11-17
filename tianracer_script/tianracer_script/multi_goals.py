#  Author: Liangqian Kong <kongliangqian@huawei.com>

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class MultiGoals(Node):
    def __init__(self):
        super().__init__("multi_goals")
        self.get_goal_paramter()
        self.retry = self.retry
        self.goalId = 0
        goalMsg = PoseStamped()
        goalMsg.header.frame_id = self.map_frame
        goalMsg.header.stamp = self.get_clock().now().to_msg()

        goalMsg.pose.orientation.w = 1.0
        goalMsg.pose.position.x = self.goalListX[self.goalId]
        goalMsg.pose.position.y = self.goalListY[self.goalId]

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self._logger.info("'NavigateToPose' action server not available, waiting...")
        goalMsg_nav = NavigateToPose.Goal()
        goalMsg_nav.pose = goalMsg

        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goalMsg_nav)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self._logger.info("Initial goal published! Goal ID is: %d" % self.goalId)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info("goal finish! Goal ID is: %d" % self.goalId)
            self.goalId = self.goalId + 1


    def get_goal_paramter(self):
        goalListX = self.get_parameter_or('~goalListX', '[0.229]')
        goalListY = self.get_parameter_or('~goalListY', '[1.713]')
        self.map_frame = self.get_parameter_or('~map_frame', 'map')
        self.retry = self.get_parameter_or('~retry', '1')

        goalListX = goalListX.replace("[", "").replace("]", "")
        goalListY = goalListY.replace("[", "").replace("]", "")
        goalListX = [float(x) for x in goalListX.split(",")]
        goalListY = [float(y) for y in goalListY.split(",")]

        if len(goalListX) == len(goalListY) and len(goalListY) >=1:
            # Constract MultiGoals Obj
            self.get_logger().info("Multi Goals Executing...")
        else:
            self.get_logger().error("Lengths of goal lists are not the same")

        self.goalListX = goalListX
        self.goalListY = goalListY


def main():
    rclpy.init()
    rclpy.spin(MultiGoals())



if __name__ == "__main__":
    main()


