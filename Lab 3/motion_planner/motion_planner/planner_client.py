import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cpp_action_pkg.action import PathPlan


class PathActionClient(Node):

    def __init__(self):
        super().__init__('planner_client')
        self._action_client = ActionClient(self, PathPlan, 'planner_server')
        self._goal_future = None
        self._get_result_future = None

    def send_goal(self, x: float, y: float) -> None:
        self.get_logger().info('Requesting path to ({}, {})'.format(x, y))

        goal_msg = PathPlan.Goal()
        goal_msg.end_position.x = x
        goal_msg.end_position.y = y

        self._action_client.wait_for_server()

        self._goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)

        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Request rejected')
            return

        self.get_logger().info('Request accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Successfully reached goal position!')
        else:
            self.get_logger().info('Did not reach goal position!')
        rclpy.shutdown()

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Intermediate position: ({}, {})'.format(
            feedback.current_position.x, feedback.current_position.y
        ))


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        raise ValueError("x and y coordinates of end poisition are required")

    action_client = PathActionClient()
    future = action_client.send_goal(float(sys.argv[1]), float(sys.argv[2]))

    rclpy.spin(action_client, future)


if __name__ == '__main__':
    main()
