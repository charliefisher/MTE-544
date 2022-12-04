import math

import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class RobotController:

    Kp_ang: float = 0.2
    EPSILON: float = 0.2  # 20cm
    X_VEL: float = 0.15

    def __init__(self, parent_node: Node) -> None:
        parent_node.create_subscription(
            Odometry, '/odom', self._receive_odom,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )   
        )
        parent_node.create_subscription(
            TFMessage, '/tf', self._receive_tf,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )   
        )
        self._vel_pub = parent_node.create_publisher(Twist, '/cmd_vel', 10)

        # transformations set by subcription to tf topic
        self._tf_odom_bf = None
        self._tf_map_odom = None

        # controller state
        self._cur_position = None
        self._cur_heading = None
        self._goal_position = None
        self._at_goal = False

        self._logger = parent_node.get_logger()

        self._logger.info('Started robot controller!')

    """Recieve the /odom message"""
    def _receive_odom(self, msg: Odometry) -> None:
        self._cur_position = msg.pose.pose.position

        # convert quarternion to robot heading
        q = msg.pose.pose.orientation
        t3 = 2.0*(q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self._cur_heading = math.atan2(t3, t4)

    """Recieve the /tf message"""
    def _receive_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                self._tf_odom_bf = self._tf_to_homogenous(tf.transform)
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self._tf_map_odom = self._tf_to_homogenous(tf.transform)

    """Convert quaternion to homogeneous transformation matrix"""
    def _tf_to_homogenous(self, transform) -> np.array:
        q1 = transform.rotation.x
        q2 = transform.rotation.y
        q3 = transform.rotation.z
        qr = transform.rotation.w

        return np.array([
            [qr**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2-q3*qr), transform.translation.x],
            [2*(qr*q3 + q1*q2), qr**2 - q1**2 + q2**2 - q3**2, transform.translation.y],
            [0, 0, 1]
        ], np.float64)

    """Read the current position of the robot"""
    @property
    def cur_position(self) -> Point:
        cur_pos_odom_np = np.array([[self._cur_position.x], [self._cur_position.y], [1]])
        
        # apply transform from odom to map
        cur_pos_map_np = self._tf_map_odom@cur_pos_odom_np
        cur_pos_map = Point()
        cur_pos_map.x = cur_pos_map_np[0, 0]
        cur_pos_map.y = cur_pos_map_np[1, 0]
        return cur_pos_map
    
    """Sets a goal position for the controller"""
    def set_goal_position(self, goal_position: Point) -> None:
        self._goal_position = goal_position

    """Checks whether the robot is at the goal position"""
    def at_goal(self) -> bool:
        assert self._goal_position is not None
        return self._at_goal

    """Stops the robot"""
    def stop(self) -> None:
        vel = Twist()
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        self._vel_pub.publish(vel)

    # !! this code must be re-entrant !!
    """Run the control loop"""
    def run(self) -> None:
        if self._goal_position is None:
            return

        assert self.cur_position is not None

        self._logger.info('position: {}'.format(self.cur_position))

        # calculate distance to goal
        x_diff = self._goal_position.x - self.cur_position.x
        y_diff = self._goal_position.y - self.cur_position.y
        euc_dist = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff,2))

        self._logger.info('distance: {}'.format(euc_dist))

        # check if we are close enough
        self._at_goal = (euc_dist <= RobotController.EPSILON)

        if self.at_goal():
            self.stop()
            return

        # compute angular error
        required_heading = math.atan2(
            self._goal_position.y - self.cur_position.y,
            self._goal_position.x - self.cur_position.x
        )
        ang_err = required_heading - self._cur_heading
        self._logger.info('cur heading: {}'.format(self._cur_heading))
        self._logger.info('required heading: {}'.format(required_heading))
        self._logger.info('ang_err: {}'.format(ang_err))

        # generate control velocity
        vel = Twist()    
        if abs(ang_err) < 0.2:
            vel.linear.x = RobotController.X_VEL
            self._logger.info('moving in x dir')
            vel.angular.z = 0.0
        else:
            vel.angular.z = RobotController.Kp_ang * ang_err
            self._logger.info('rotating')
            vel.linear.x = 0.0
        self._vel_pub.publish(vel)
