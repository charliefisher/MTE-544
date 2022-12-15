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

# dictionary keys for transforms
BASE_FOOTPRINT_TO_ODOM: str = 'base_footprint_to_odom'
LEFT_WHEEL_TO_BASE_LINK: str = 'left_wheel_to_base'
RIGHT_WHEEL_TO_BASE_LINK: str = 'right_wheel_to_base'


"""
Kalman Filter to localize turtlebot 4
State vector: [x; x_dot; theta; omega]
"""
class KalmanFilter:
    def __init__(self, data_file_path: str) -> None:

        # read data from bagfile
        with open(data_file_path, 'r') as data_file:
            data = json.load(data_file)

        self._imu_data = data['imu']['data']
        self._tf_data = data['tf']

        self._dt = 0.1  # rate of Kalman filter

        # set when KalmanFilter::run is called
        self._t_final: float = None
        self._T: np.array = None
        self._state: States = None
        self._measurement: np.array = None

        self.n_states = 5

        # prediction estimation - this is your "Priori"
        self.xhat = np.array([0, 0, 0, 0, 0]).reshape((self.n_states, 1))
        self.P = np.identity(self.n_states) # covariance initial estimation between the
        # the covariance matrix P is the weighted covariance between the
        # motion model definition - establish your robots "movement"

        # state space equations of system (in robot frame)
        # state vector: [x, x_dot, x_double_dot, theta, omega] 
        # input vector: [a_x, alpha]
        # with respect to robot frame, y_dot (and y_double_dot) is always 0
        self.A = np.array([
            [1, self._dt, 1/4*self._dt**2, 0, 0],
            [0, 1, self._dt/2, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 1, self._dt/2],
            [0, 0, 0, 0, 0]
        ])  # state transition
        self.B = np.array([
            [1/4*self._dt**2, 0],
            [1/2*self._dt, 0],
            [1, 0],
            [0, 1/2*self._dt],
            [0, 1]
        ])  # input
        
        # this is the model variance/covariance (usually we treat it as the input matrix squared).
        # these are the discrete components, found by performing a taylor's
        # expansion on the continuous model
        var_a_x = self._imu_data[0]['linear_acceleration_covariance'][0]  # var of x about x axis
        var_omega = self._imu_data[0]['angular_velocity_covariance'][8]  # var of z about z axis

        Q_a = np.zeros((self.n_states, self.n_states))
        Q_a[2,2] = var_a_x
        Q_a[4,4] = var_omega

        self.Q = self.A@Q_a@self.A.transpose()

        ## Measurement Model - Update component
        wheel_diameter = 0.072  # 72 mm
        track = (self._tf_data[LEFT_WHEEL_TO_BASE_LINK][0]['translation'][1] -
            self._tf_data[RIGHT_WHEEL_TO_BASE_LINK][0]['translation'][1])
        assert track > 0

        # what this represents is our "two" sensors, both with linear relationships
        # to position and velocity respectively
        self.C = np.array([
            [self._dt/2, 1/2, 0, -self._dt/track, -1/track],
            [self._dt/2, 1/2, 0, self._dt/track, 1/track],
        ])*wheel_diameter/2

        # in actual environments, what this does is translate our measurement to a
        # voltage or some other metric that can be ripped directly from the sensor
        # when taking online measurements. We compare those values as our "error"

        self.R = np.array([[0.05, 0], [0, 0.05]]) # this is the sensor model variance-usually characterized to accompany
                                                  # the sensor model already starting

    def run(self, t_final: float) -> States:      
        # store timesteps of particle filter
        # TODO: maybe update this with actual times of particle filter
        self._t_final = t_final
        self._T = np.arange(0, self._t_final, self._dt)

        # store state for each iteration of particle filter
        self._state = np.zeros((len(self._T) + 1, self.n_states))
        self._measurement = np.zeros((len(self._T) + 1, 2))

        for k in range(len(self._T)):
            imu_measurement = self._imu_data[k]
            a_x = imu_measurement['linear_acceleration'][0]
            omega = imu_measurement['angular_velocity'][2]

            u = np.array([a_x, omega]).reshape((2, 1))

            # read measurement
            x_pos = 0.03
            y_pos = 0
            self._measurement[k, :] = np.array([x_pos, y_pos]).reshape((1, 2))

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

            y_k = self._measurement[k, :].T.reshape((2,1))
            self.xhat = xhat_k + K@(y_k - self.C@xhat_k)
            self.P = (1 - K@self.C)@P_predict

            self._state[k, :] = self.xhat.T  # store state

        return self._state

    """Convert orientation quaternion to robot heading"""
    def orientation_to_heading(self, orientation: Quaternion) -> float:
        # convert quarternion to robot heading
        q = orientation
        t3 = 2.0*(q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return math.atan2(t3, t4)

    """Convert quaternion to homogeneous transformation matrix"""
    def tf_to_homogenous(self, translation: Vector3, rotation: Quaternion) -> np.array:
        q1 = rotation[0]  # x
        q2 = rotation[1]  # y
        q3 = rotation[2]  # z
        qr = rotation[3]  # w

        return np.array([
            [qr**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2-q3*qr), translation[0]],
            [2*(qr*q3 + q1*q2), qr**2 - q1**2 + q2**2 - q3**2, translation[1]],
            [0, 0, 1]
        ], np.float64)

    """Calculate the actual pose of the robot from tf from base footprint to odom"""
    def _compute_true_robot_position(self) -> np.array:
        tf_base_footprint_to_odom = self._tf_data[BASE_FOOTPRINT_TO_ODOM]
        n_points = len(tf_base_footprint_to_odom)  # number of measurements
        robot_pose = np.zeros((n_points, 3))

        origin = np.array([0, 0, 1])  # homogenous coordinate of origin

        for i, tf in enumerate(tf_base_footprint_to_odom):
            # compute transform for current pose
            bf_to_odom = self.tf_to_homogenous(tf['translation'], tf['rotation'])
            # transform origin in base footprint frame to odom frame
            bf_pose_in_odom = bf_to_odom@origin
            # store robot pose in odom frame
            robot_pose[i] = bf_pose_in_odom

        return robot_pose

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
        true_robot_pose = self._compute_true_robot_position()
        plt.scatter(true_robot_pose[:, 0], true_robot_pose[:, 1])

        # calculate the estimated pose in the world frame
        robot_pose = self._transform_state_to_world()
        plt.scatter(robot_pose[:, 0], robot_pose[:, 1])

        # calculate the measured pose in the world frame
        measured_pose = self._transform_state_to_world()
        plt.scatter(measured_pose[:, 0], measured_pose[:, 1])

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
