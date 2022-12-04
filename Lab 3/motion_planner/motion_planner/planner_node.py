import math
import time
from typing import Tuple

import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

from cpp_action_pkg.action import PathPlan

from motion_planner.controller import RobotController
from motion_planner.a_star import astar


import matplotlib.pyplot as plt # TODO: remove me


class PlannerServer(Node):

    def __init__(self):
        super().__init__('planner_server')
        self._action_server = ActionServer(
            self,
            PathPlan,
            'planner_server',
            self._execute_callback)

        self._costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._costmap_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
        )

        self._costmap = None
        self._costmap_width = None
        self._costmap_height = None
        self._costmap_resolution = None
        self._costmap_origin = None
        self._costmap_downsample_factor: int = 1

        self._controller = RobotController(self)
        
        self.get_logger().info('Started planner server!')

    def _visualize_costmap(self) -> None:
        x = np.arange(
            self._costmap_origin.position.x,
            self._costmap_origin.position.x + self._costmap_width * self._costmap_resolution,
            self._costmap_resolution 
        )
        y = np.arange(
            self._costmap_origin.position.y,
            self._costmap_origin.position.y + self._costmap_height * self._costmap_resolution,
            self._costmap_resolution
        )
        X, Y = np.meshgrid(x, y)

        plt.figure()
        plot = plt.pcolormesh(X, Y, self._costmap)
        plt.colorbar(plot)
        plt.grid()

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self._costmap_width = msg.info.width
        self._costmap_height = msg.info.height
        self._costmap_resolution = msg.info.resolution
        self._costmap_origin = msg.info.origin
        self._costmap = np.array(msg.data, dtype=np.int8).reshape(
            (self._costmap_height, self._costmap_width)
        )
        # self.get_logger().info('Received costmap')

        # downsample costmap
        self._costmap = self._costmap[
            ::self._costmap_downsample_factor,
            ::self._costmap_downsample_factor
        ]
        self._costmap_width //= self._costmap_downsample_factor
        self._costmap_height //= self._costmap_downsample_factor
        self._costmap_resolution *= self._costmap_downsample_factor
        # self.get_logger().info('Downsampled costmap by {}'.format(self._costmap_downsample_factor))

    """Convert (x_idx, y_idx) of costmap to (x, y) coordinate in world space"""
    def _costmap_to_world(self, x_idx: int, y_idx: int) -> np.array:
        return np.array([
            self._costmap_origin.position.x + x_idx * self._costmap_resolution + 0.5*self._costmap_resolution,
            self._costmap_origin.position.y + y_idx * self._costmap_resolution + 0.5*self._costmap_resolution
        ])

    """Convert (x, y) coordinate in world space to (x_idx, y_idx) of costmap"""
    def _world_to_costmap(self, x: float, y: float) -> Tuple[int, int]:
        return (
            math.floor((x - self._costmap_origin.position.x) / self._costmap_resolution),
            math.floor((y - self._costmap_origin.position.y) / self._costmap_resolution)
        )

    def _set_controller_goal(self, path: np.array, pt_idx: int) -> None:
        next_pt = path[pt_idx]
        intermediate_goal = Point()
        intermediate_goal.x = next_pt[0]
        intermediate_goal.y = next_pt[1]

        self.get_logger().info('Next point is: {}'.format(intermediate_goal))
        self._controller.set_goal_position(intermediate_goal)

    def _execute_callback(self, request_handle: PathPlan):
        self.get_logger().info('Showing costmap...')
        self._visualize_costmap()
        plt.show()

        if self._costmap is None:
            self.get_logger().error('Waiting for costmap')
            result = PathPlan.Result()
            result.success = False
            return result
        
        # Define here your start and end points
        start = self._world_to_costmap(self._controller.cur_position.x, self._controller.cur_position.y)
        # start = self._world_to_costmap(0, 0)
        self.get_logger().info('start pos {} {}'.format(self._controller.cur_position.x, self._controller.cur_position.y))
        self.get_logger().info('start pos (in costmap) {} {}'.format(start[0], start[1]))
        
        end = self._world_to_costmap(request_handle.request.end_position.x, request_handle.request.end_position.y)
        greediness = 1

        self.get_logger().info('Planning path from ({}, {}) to ({}, {})'.format(
            start[0], start[1], end[0], end[1]
        ))

        # Compute the path with your implementation of Astar
        path = np.asarray(astar(self._costmap.T, start, end, greediness), dtype=np.float)

        self.get_logger().info('Full path: {}'.format(path))

        # path is empty, could not find a path to the goal
        if len(path) == 0:
            self.get_logger().error('Could not find path to goal')
            result = PathPlan.Result()
            result.success = False
            return result


        self.get_logger().info('Full path (costmap): {}'.format(path))
        path = np.array([self._costmap_to_world(*p) for p in path])
        self.get_logger().info('Full path (world): {}'.format(path))

        self._visualize_costmap()
        plt.plot(path[:, 0], path[:, 1], 'r')
        plt.show()

        finished_control = False
        self._set_controller_goal(path, 0)
        pt_idx = 1
        def execute_path() -> None:
            nonlocal finished_control, pt_idx

            if self._controller.at_goal():
                # print feedback message
                feedback_msg = PathPlan.Feedback()
                feedback_msg.current_position = self._controller.cur_position
                request_handle.publish_feedback(feedback_msg)

                # send next intermediate goal
                self._set_controller_goal(path, pt_idx)
                pt_idx += 1
                
                finished_control = pt_idx >= len(path)

            self._controller.run()

        controller_timer_period = 0.1  # seconds
        path_exec_timer = self.create_timer(controller_timer_period, execute_path)

        # spin to execute path and get updates on subscribers
        while not finished_control:
            rclpy.spin_once(self)

        self._controller.stop()
        path_exec_timer.cancel()

        time.sleep(0)  # yield thread

        request_handle.succeed()
        result = PathPlan.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    planner_server = PlannerServer()

    rclpy.spin(planner_server)


if __name__ == '__main__':
    main()
