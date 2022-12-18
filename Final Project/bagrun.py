import json

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

# import messages
from builtin_interfaces.msg import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


# total number of messages per topic
TF_COUNT: int = 1287
TF_STATIC_COUNT: int = 1
IMU_COUNT: int = 5280
ODOM_COUNT: int = 776

# keys for specific transforms of interest
BASE_FOOTPRINT_TO_ODOM: str = 'base_footprint_to_odom'
LEFT_WHEEL_TO_BASE_LINK: str = 'left_wheel_to_base'
RIGHT_WHEEL_TO_BASE_LINK: str = 'right_wheel_to_base'

class Bagreader(Node):

    def __init__(self, export_file):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('bagreader')

        self.get_logger().info('Reading bag file...')

        self.export_file = export_file

        # create the subscriber objects
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.imu_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # initialize data dictionary, populated by subscriber callbacks
        self.data = {
            'tf': {
                BASE_FOOTPRINT_TO_ODOM: [],
                LEFT_WHEEL_TO_BASE_LINK: [],
                RIGHT_WHEEL_TO_BASE_LINK: [],
            },
            'tf_static': {},
            'imu': {},
            'odom': {},
        }
        # count how many of each message type have been read
        self.tf_msg_count = 0
        self.tf_static_msg_count = 0
        self.imu_msg_count = 0
        self.odom_msg_count = 0

    """Serializes a Vector3 message"""
    def serialize_vector3(self, vec: Vector3) -> tuple:
        return (vec.x, vec.y, vec.z)

    """Serializes a Quaternion message"""
    def serialize_quaternion(self, q: Quaternion) -> tuple:
        return (q.x, q.y, q.z, q.w)

    """Serializes a Transform message"""
    def serialize_transform(self, transform: Transform) -> dict:
        return {
            'translation': self.serialize_vector3(transform.translation),
            'rotation': self.serialize_quaternion(transform.rotation),
        }

    """Serializes a Time message"""
    def serialize_time(self, time: Time) -> int:
        return rclpy.time.Time.from_msg(time).nanoseconds

    def tf_callback(self, msg: TFMessage) -> None:
        # add only transforms of interest
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                self.data['tf'][BASE_FOOTPRINT_TO_ODOM].append({
                    'time': self.serialize_time(tf.header.stamp),
                    'transform': self.serialize_transform(tf.transform),
                })
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'wheel_left_link':
                self.data['tf'][LEFT_WHEEL_TO_BASE_LINK].append({
                    'time': self.serialize_time(tf.header.stamp),
                    'transform': self.serialize_transform(tf.transform),
                })
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'wheel_right_link':
                self.data['tf'][RIGHT_WHEEL_TO_BASE_LINK].append({
                    'time': self.serialize_time(tf.header.stamp),
                    'transform': self.serialize_transform(tf.transform),
                })

        self.tf_msg_count += 1
        self.check_complete()

    def tf_static_callback(self, msg: TFMessage) -> None:
        # keys for specific transforms of interest
        BASE_LINK_TO_BASE_FOOTPRINT: str = 'base_link_to_base_footprint'
        IMU_LINK_TO_BASE_LINK: str = 'imu_link_to_base_link'

        # add only transforms of interest
        for tf in msg.transforms:
            if tf.header.frame_id == 'base_footprint' and tf.child_frame_id == 'base_link':
                self.data['tf_static'][BASE_LINK_TO_BASE_FOOTPRINT] = self.serialize_transform(tf.transform)
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'imu_link':
                self.data['tf_static'][IMU_LINK_TO_BASE_LINK] = self.serialize_transform(tf.transform)

        self.tf_static_msg_count += 1
        self.check_complete()

    def imu_callback(self, msg: Imu) -> None:
        # add frame to imu data once
        if 'frame' not in self.data['imu']:
            self.data['imu']['frame'] = msg.header.frame_id
        else:
            assert(self.data['imu']['frame'] == msg.header.frame_id)
        
        # add covariance to imu data once
        if 'orientation_covariance' not in self.data['imu']:
            self.data['imu']['orientation_covariance'] = msg.orientation_covariance.tolist()
        if 'angular_velocity_covariance' not in self.data['imu']:
            self.data['imu']['angular_velocity_covariance'] = msg.angular_velocity_covariance.tolist()
        if 'linear_acceleration_covariance' not in self.data['imu']:
            self.data['imu']['linear_acceleration_covariance'] = msg.linear_acceleration_covariance.tolist()
        
        # initialize data field once
        if 'data' not in self.data['imu']:
            self.data['imu']['data'] = []

        # add imu reading
        self.data['imu']['data'].append({
            'time': self.serialize_time(msg.header.stamp),
            'orientation': self.serialize_quaternion(msg.orientation),
            'angular_velocity': self.serialize_vector3(msg.angular_velocity),
            'linear_acceleration': self.serialize_vector3(msg.linear_acceleration),
        })

        self.imu_msg_count += 1
        self.check_complete()

    def odom_callback(self, msg: Odometry) -> None:
        # add frame to odom data once
        if 'frame' not in self.data['odom']:
            self.data['odom']['frame'] = msg.header.frame_id
        else:
            assert(self.data['odom']['frame'] == msg.header.frame_id)
        
        # add covariance to odom data once
        if 'pose_covariance' not in self.data['odom']:
            self.data['odom']['pose_covariance'] = msg.pose.covariance.tolist()
        if 'twist_covariance' not in self.data['odom']:
            self.data['odom']['twist_covariance'] = msg.twist.covariance.tolist()
        
        # initialize data field once
        if 'data' not in self.data['odom']:
            self.data['odom']['data'] = []

        # add odom reading
        self.data['odom']['data'].append({
            'time': self.serialize_time(msg.header.stamp),
            'pose': {
                'position': self.serialize_vector3(msg.pose.pose.position),
                'orientation': self.serialize_quaternion(msg.pose.pose.orientation),
            },
            'twist': {
                'linear': self.serialize_vector3(msg.twist.twist.linear),
                'angular': self.serialize_vector3(msg.twist.twist.angular),
            },
        })

        self.odom_msg_count += 1
        self.check_complete()

    """Checks if bag reading is complete and exports data"""
    def check_complete(self) -> None:
        # get total message count and print update
        total_msg_count: int = (self.tf_msg_count + self.tf_static_msg_count + 
            self.imu_msg_count + self.odom_msg_count)
        if total_msg_count % 500 == 0:
            self.get_logger().info('Read {} messages'.format(total_msg_count))

        # if all messages have been read, export to file
        if (self.tf_msg_count == TF_COUNT and
            self.tf_static_msg_count == TF_STATIC_COUNT and
            self.imu_msg_count == IMU_COUNT and
            self.odom_msg_count == ODOM_COUNT):
            
            # export data to json
            self.get_logger().info('Exporting data...')
            with open(self.export_file, 'w') as fp:
                json.dump(self.data, fp)

            self.get_logger().info('Reading complete!')


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    bagreader = Bagreader('/home/fisher_charlie/mte544_cw/final_proj/path.json')
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(bagreader)
    # stop node
    bagreader.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
