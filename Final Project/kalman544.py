#!/usr/bin/env python3

"""
Kalman Filter implementation based on example code provided on LEARN
"""

import json
import math

import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt

# custom type hints
Quaternion = np.array
Vector3 = np.array
States = np.array
Measurement = np.array

# dictionary keys for transforms
BASE_FOOTPRINT_TO_ODOM: str = 'base_footprint_to_odom'
LEFT_WHEEL_TO_BASE_LINK: str = 'left_wheel_to_base'
RIGHT_WHEEL_TO_BASE_LINK: str = 'right_wheel_to_base'


"""
Kalman Filter to localize turtlebot 4
State vector: [x; x_dot; y; y_dot; theta]
"""
class KalmanFilter:
    N_STATES: int = 5
    
    def __init__(self, data_file_path: str) -> None:
        # read exported data from bagfile
        with open(data_file_path, 'r') as data_file:
            data = json.load(data_file)

        self._imu = data['imu']
        self._imu_data = self._imu['data']
        self._odom = data['odom']
        self._odom_data = self._odom['data']
        self._tf_data = data['tf']

        # imu messages posted at 5ms intervals
        # tf and odom messages posted at 34ms is 34
        # run the Kalman Filter at the slower of the two, 34 ms or
        self._dt = 0.034  # rate of Kalman filter (seconds)

        # set when KalmanFilter::run is called
        self._t_final: float = None
        self._T: np.array = None
        self._state: States = None
        self._measurement: np.array = None

        # prediction estimation - this is your "Priori"
        self.xhat = np.array([0, 0, 0, 0, 0]).reshape((KalmanFilter.N_STATES, 1))
        self.P = np.identity(KalmanFilter.N_STATES) # covariance initial estimation between the
        # the covariance matrix P is the weighted covariance between the
        # motion model definition - establish your robots "movement"

        # state space equations of system (in robot frame)
        # state vector: [x, x_dot, x_double_dot, theta, omega] 
        # input vector: [a_x, alpha]
        # with respect to robot frame, y_dot (and y_double_dot) is always 0
        self.A = np.array([
            [1, self._dt, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, self._dt, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])  # state transition
        self.B = np.array([
            [1/2*self._dt**2, 0, 0],
            [self._dt, 0, 0],
            [0, 1/2*self._dt**2, 0],
            [0, self._dt, 0],
            [0, 0, self._dt]
        ])  # input
        
        # this is the model variance/covariance (usually we treat it as the input matrix squared).
        # these are the discrete components, found by performing a taylor's
        # expansion on the continuous model
        var_a_x = self._imu['linear_acceleration_covariance'][0]  # var of x about x axis
        var_a_y = self._imu['linear_acceleration_covariance'][4]  # var of y about y axis
        var_a = np.sqrt(var_a_x**2 + var_a_y**2)
        var_omega = self._imu['angular_velocity_covariance'][8]  # var of z about z axis

        Q_a = np.array([
            [1/2*self._dt**2*var_a, 0, 0, 0, 0],
            [0, self._dt*var_a, 0, 0, 0],
            [0, 0, 1/2*self._dt**2*var_a, 0, 0],
            [0, 0, self._dt*var_a, 0, 0],
            [0, 0, 0, 0, self._dt*var_omega],
        ])
        self.Q = self.A@Q_a@self.A.transpose()

        ## Measurement Model

        # states track x and y directly already, no explicit motion model
        self.C = np.identity(5)

        # in actual environments, what this does is translate our measurement to a
        # voltage or some other metric that can be ripped directly from the sensor
        # when taking online measurements. We compare those values as our "error"

        # note that both pose_covariance and twist_covariance are diagonal matrices
        # thus, R will also be a diagonal matrix
        pose_covariance = np.array(self._odom['pose_covariance']).reshape((6,6))
        twist_covariance = np.array(self._odom['twist_covariance']).reshape((6,6))
        assert(np.array_equal(pose_covariance, np.diagflat(np.diag(pose_covariance))))
        assert(np.array_equal(twist_covariance, np.diagflat(np.diag(twist_covariance))))

        cov_x_x = pose_covariance[0,0]
        cov_y_y = pose_covariance[1,1]
        cov_rot_z_z = pose_covariance[5,5]
        cov_xd_xd = twist_covariance[0,0]
        cov_yd_yd = twist_covariance[1,1]

        self.R = np.zeros((KalmanFilter.N_STATES, KalmanFilter.N_STATES))
        self.R[0,0] = cov_x_x
        self.R[1,1] = cov_xd_xd
        self.R[2,2] = cov_y_y
        self.R[3,3] = cov_yd_yd
        self.R[4,4] = cov_rot_z_z
        assert(np.array_equal(self.R, np.diagflat(np.diag(self.R))))

        # self.R = np.array([[0.05, 0], [0, 0.05]]) # this is the sensor model variance-usually characterized to accompany
                                                  # the sensor model already starting

    """Finds the most recent measurement from /odom given a time"""
    def _get_closest_measurement(self, t: float) -> Measurement:
        closest_measurement = None

        measurement_start_time = self._odom_data[0]['time']
        for measurument in self._odom_data:
            elapsed_time = (measurument['time'] - measurement_start_time) / (1e9)
            if elapsed_time <= t:
                closest_measurement = measurument
            else:
                break

        assert(closest_measurement is not None)

        # extract state variables from measurement
        x = closest_measurement['pose']['position'][0]
        y = closest_measurement['pose']['position'][1]
        omega = self.orientation_to_heading(closest_measurement['pose']['orientation'])
        x_dot = closest_measurement['twist']['linear'][0]
        y_dot = closest_measurement['twist']['linear'][1]
        return np.array([x, x_dot, y, y_dot, omega]).reshape((KalmanFilter.N_STATES, 1))

    def run(self, t_final: float) -> States:      
        # store timesteps of particle filter
        # TODO: maybe update this with actual times of particle filter
        self._t_final = t_final
        self._T = np.arange(0, self._t_final, self._dt)

        # store state for each iteration of particle filter
        self._state = np.zeros((len(self._T) + 1, KalmanFilter.N_STATES))
        self._measurement = np.zeros((len(self._T) + 1, KalmanFilter.N_STATES))

        for k, t in enumerate(self._T):
            imu_measurement = self._imu_data[k]
            a_x = imu_measurement['linear_acceleration'][0]
            a_y = imu_measurement['linear_acceleration'][1]
            omega = imu_measurement['angular_velocity'][2]

            u = np.array([a_x, a_y, omega]).reshape((3, 1))

            # read measurement
            y_k = self._get_closest_measurement(t)
            self._measurement[k, :] = y_k.T

            #########################################
            ###### Kalman Filter Estimation #########
            #########################################
            # Prediction update
            xhat_k = self.A@self.xhat + self.B@u # we do not put noise on our prediction
            P_predict = self.A@self.P@self.A.transpose() + self.Q
            # this co-variance is the prediction of essentially how the measurement and sensor model move together
            # in relation to each state and helps scale our kalman gain by giving
            # the ratio. By Definition, P is the variance of the state space, and
            # by applying it to the motion model we're getting a motion uncertainty
            # which can be propogated and applied to the measurement model and
            # expand its uncertainty as well

            # Measurement Update and Kalman Gain
            K = P_predict@self.C.transpose()@np.linalg.inv(self.C@P_predict@self.C.transpose() + self.R)
            # the pseudo inverse of the measurement model, as it relates to the model covariance
            # if we don't have a measurement for velocity, the P-matrix tells the
            # measurement model how the two should move together (and is normalised
            # in the process with added noise), which is how the kalman gain is
            # created --> detailing "how" the error should be scaled based on the
            # covariance. If you expand P_predict out, it's clearly the
            # relationship and cross-projected relationships, of the states from a
            # measurement and motion model perspective, with a moving scalar to
            # help drive that relationship towards zero (P should stabilise).

            self.xhat = xhat_k + K@(y_k - self.C@xhat_k)
            self.P = (np.identity(KalmanFilter.N_STATES) - K@self.C)@P_predict

            self._state[k, :] = self.xhat.T  # store state

        return self._state

    """Convert orientation quaternion to robot heading"""
    def orientation_to_heading(self, orientation: Quaternion) -> float:
        qx = orientation[0]
        qy = orientation[1]
        qz = orientation[2]
        qw = orientation[3]

        # convert quarternion to robot heading
        t3 = 2.0*(qw * qz + qx * qy)
        t4 = 1.0 - 2.0*(qy*qy + qz*qz)
        return math.atan2(t3, t4)

    """Convert quaternion to homogeneous transformation matrix"""
    def tf_to_homogenous(self, translation: Vector3, rotation: Quaternion) -> np.array:
        qx = rotation[0]
        qy = rotation[1]
        qz = rotation[2]
        qw = rotation[3]

        return np.array([
            [qw**2 + qx**2 - qy**2 - qz**2, 2*(qx*qy-qz*qw), translation[0]],
            [2*(qw*qz + qx*qy), qw**2 - qx**2 + qy**2 - qz**2, translation[1]],
            [0, 0, 1]
        ], np.float64)

    """Calculate the actual pose of the robot from tf from base footprint to odom"""
    def _compute_true_robot_position(self) -> tuple[np.array, np.array]:
        tf_base_footprint_to_odom = self._tf_data[BASE_FOOTPRINT_TO_ODOM]
        n_points = len(tf_base_footprint_to_odom)  # number of measurements
        t = np.zeros((n_points, 1))
        robot_pose = np.zeros((n_points, 3))

        origin = np.array([0, 0, 1])  # homogenous coordinate of origin

        for i, tf in enumerate(tf_base_footprint_to_odom):
            # compute transform for current pose
            bf_to_odom = self.tf_to_homogenous(
                tf['transform']['translation'], tf['transform']['rotation']
            )
            # transform origin in base footprint frame to odom frame
            bf_pose_in_odom = bf_to_odom@origin
            # store robot pose in odom frame
            robot_pose[i] = bf_pose_in_odom
            t[i] = (tf['time'] - tf_base_footprint_to_odom[0]['time']) / (1e9)

        return (t, robot_pose)

    def _transform_state_to_world(self) -> np.array:
        pose = np.zeros((len(self._state), 3))

        for i in range(1, len(self._state)):
            prev_state = self._state[i-1,:].T
            cur_state = self._state[i,:].T

            pose[i, 0] = cur_state[0]
            pose[i, 1] = pose[i-1, 1] + (prev_state[1]*self._dt*np.sin(prev_state[3]))
            pose[i, 2] = cur_state[3]
        
        return pose

    def _transform_measured_to_world(self) -> np.array:
        pose = np.zeros((len(self._measurement), 3))

        for i in range(1, len(self._measurement)):
            prev_measure = self._measurement[i-1,:].T
            cur_measure = self._measurement[i,:].T

            pose[i, 0] = cur_measure[0]
            pose[i, 1] = pose[i-1, 1] + (prev_measure[1]*self._dt)
        
        return pose

    def plot_results(self):

        plt.figure()

        # calculate actual robot pose, used to calculate MSE
        t, true_robot_pose = self._compute_true_robot_position()
        plt.scatter(true_robot_pose[:, 0], true_robot_pose[:, 1], c=t, s=5)
        plt.colorbar(label='Time (s)')

        # calculate the estimated pose in the world frame
        robot_pose = self._transform_state_to_world()
        plt.scatter(self._state[:, 0], self._state[:, 2], c='r', s=5)

        # # calculate the measured pose in the world frame
        # measured_pose = self._transform_state_to_world()
        # plt.scatter(measured_pose[:, 0], measured_pose[:, 1])

        plt.legend([
            'true position',
            'predicted position',
            'measured position',
        ])

        plt.show()

def main():
    print("MTE544 Final Project - Kalman Filter")
    kf = KalmanFilter('path.json')
    kf.run(10)
    kf.plot_results()

if __name__ == '__main__':
    main()
