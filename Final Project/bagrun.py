import json

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

# import messages
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Quaternion
from sensor_msgs.msg import Imu, JointState


TF_COUNT: int = 1287
TF_STATIC_COUNT: int = 1
IMU_COUNT: int = 5280
ENCODER_COUNT: int = 776

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
            JointState, '/joint_states', self.encoder_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.data = {
            'tf': {
                BASE_FOOTPRINT_TO_ODOM: [],
                LEFT_WHEEL_TO_BASE_LINK: [],
                RIGHT_WHEEL_TO_BASE_LINK: [],
            },
            'tf_static': {},
            'imu': {},
            'encoder': {},
        }
        self.tf_msg_count = 0
        self.tf_static_msg_count = 0
        self.imu_msg_count = 0
        self.encoder_msg_count = 0

    def serialize_vector3(self, vec: Vector3) -> tuple:
        return (vec.x, vec.y, vec.z)

    def serialize_quaternion(self, q: Quaternion) -> tuple:
        return (q.x, q.y, q.z, q.w)

    def serialize_transform(self, transform: Transform) -> dict:
        return {
            'translation': self.serialize_vector3(transform.translation),
            'rotation': self.serialize_quaternion(transform.rotation),
        }

    def tf_callback(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                self.data['tf'][BASE_FOOTPRINT_TO_ODOM].append(
                    self.serialize_transform(tf.transform)
                )
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'wheel_left_link':
                self.data['tf'][LEFT_WHEEL_TO_BASE_LINK].append(
                    self.serialize_transform(tf.transform)
                )
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'wheel_right_link':
                self.data['tf'][RIGHT_WHEEL_TO_BASE_LINK].append(
                    self.serialize_transform(tf.transform)
                )

        self.tf_msg_count += 1
        self.check_complete()

    def tf_static_callback(self, msg: TFMessage) -> None:
        BASE_LINK_TO_BASE_FOOTPRINT: str = 'base_link_to_base_footprint'
        IMU_LINK_TO_BASE_LINK: str = 'imu_link_to_base_link'

        for tf in msg.transforms:
            if tf.header.frame_id == 'base_footprint' and tf.child_frame_id == 'base_link':
                self.data['tf_static'][BASE_LINK_TO_BASE_FOOTPRINT] = self.serialize_transform(tf.transform)
            if tf.header.frame_id == 'base_link' and tf.child_frame_id == 'imu_link':
                self.data['tf_static'][IMU_LINK_TO_BASE_LINK] = self.serialize_transform(tf.transform)

        self.tf_static_msg_count += 1
        self.check_complete()

    def imu_callback(self, msg: Imu) -> None:
        if 'frame' not in self.data['imu']:
            self.data['imu']['frame'] = msg.header.frame_id
        else:
            assert(self.data['imu']['frame'] == msg.header.frame_id)
        
        if 'data' not in self.data['imu']:
            self.data['imu']['data'] = []

        self.data['imu']['data'].append({
            'orientation': self.serialize_quaternion(msg.orientation),
            'orientation_covariance': msg.orientation_covariance.tolist(),
            'angular_velocity': self.serialize_vector3(msg.angular_velocity),
            'angular_velocity_covariance': msg.angular_velocity_covariance.tolist(),
            'linear_acceleration': self.serialize_vector3(msg.linear_acceleration),
            'linear_acceleration_covariance': msg.linear_acceleration_covariance.tolist(),
        })

        self.imu_msg_count += 1
        self.check_complete()

    def encoder_callback(self, msg: JointState) -> None:
        for i, enc_name in enumerate(msg.name):
            if enc_name not in self.data['encoder']:
                self.data['encoder'][enc_name] = {
                    'position': [],
                    'velocity': [],
                    'effort': [],
                }

            if len(msg.position) > 0:
                self.data['encoder'][enc_name]['position'].append(msg.position[i])
            if len(msg.velocity) > 0:
                self.data['encoder'][enc_name]['velocity'].append(msg.velocity[i])
            if len(msg.effort) > 0:
                self.data['encoder'][enc_name]['effort'].append(msg.effort[i])

        self.encoder_msg_count += 1
        self.check_complete()

    def check_complete(self) -> None:
        total_msg_count: int = self.tf_msg_count + self.tf_static_msg_count + self.imu_msg_count + self.encoder_msg_count
        if total_msg_count % 500 == 0:
            self.get_logger().info('Read {} messages'.format(total_msg_count))

        # self.get_logger().info('{} {} {}'.format(self.tf_msg_count, self.imu_msg_count, self.encoder_msg_count))

        if (self.tf_msg_count == TF_COUNT and
            self.tf_static_msg_count == TF_STATIC_COUNT and
            self.imu_msg_count == IMU_COUNT and
            self.encoder_msg_count == ENCODER_COUNT):
            
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
