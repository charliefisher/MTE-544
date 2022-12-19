#!/usr/bin/env python3

"""
Kalman filter implementation based on example code provided on LEARN
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


"""
Kalman filter to localize TurtleBot 3
"""
class KalmanFilter:
    N_STATES: int = 5
    
    def __init__(self, data_file_path: str) -> None:
        # read exported data from bag file
        with open(data_file_path, 'r') as data_file:
            data = json.load(data_file)

        self._imu = data['imu']
        self._imu_data = self._imu['data']
        self._odom = data['odom']
        self._odom_data = self._odom['data']
        self._tf_data = data['tf']

        # imu messages posted at 5ms intervals
        # tf and odom messages posted at 34ms is 34
        # run the Kalman filter at the slower of the two, 34 ms or 0.034 s
        self._dt = 0.034  # rate of Kalman filter (seconds)

        # set when KalmanFilter::run is called
        # stores the timestamp (starting from zero) and the state of each iteration
        self._T: np.array = None
        self._state: States = None

        ## Motion Model ##

        # state space equations of system (in global frame)
        # state vector: [x; x_dot; y; y_dot; theta] 
        # input vector: [a_x; a_y; omega]
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
        
        # model covariance
        var_a = self._imu['linear_acceleration_covariance'][0]  # var of x about x axis
        var_omega = self._imu['angular_velocity_covariance'][8]  # var of z about z axis
        # NOTE: var_a and var_omega are in the robots frame
        # The angular velocity in the robots frame is equal to the angular
        # velocity in the global frame
        # The linear acceleration in the robots frame is decomposed into its global
        # x and y components
        # The robot acceleration in y in the robot frame is 0 since the TurtleBot 3 is
        # nonholonomic

        # cov(x, x_dot) = (1)*sigma_x*sigma_x_dot
        # assume variance of y is the same as x
        # assume variance of x and y do not depend on theta
        self.Q = np.array([
            [1/4*self._dt**4*var_a, 1/2*self._dt**3*var_a, 0, 0, 0],
            [1/2*self._dt**3*var_a, self._dt**2*var_a, 0, 0, 0],
            [0 , 0, 1/4*self._dt**4*var_a, 1/2*self._dt**3*var_a, 0],
            [0, 0, 1/2*self._dt**3*var_a, self._dt**2*var_a, 0],
            [0, 0, 0, 0, self._dt**2*var_omega],
        ])

        ## Measurement Model ##

        # measurement tracks state variables directly already
        # no explicit measurement model
        self.C = np.identity(5)

        # NOTE: both pose_covariance and twist_covariance are diagonal matrices
        # thus, R will also be a diagonal matrix
        pose_covariance = np.array(self._odom['pose_covariance']).reshape((6,6))
        twist_covariance = np.array(self._odom['twist_covariance']).reshape((6,6))
        assert(np.array_equal(pose_covariance, np.diagflat(np.diag(pose_covariance))))
        assert(np.array_equal(twist_covariance, np.diagflat(np.diag(twist_covariance))))

        cov_x_x = pose_covariance[0,0]
        cov_y_y = pose_covariance[1,1]
        cov_rot_z_z = twist_covariance[5,5]
        cov_xd_xd = twist_covariance[0,0]
        cov_yd_yd = twist_covariance[1,1]

        # assumes measurements are independent
        self.R = np.zeros((KalmanFilter.N_STATES, KalmanFilter.N_STATES))
        self.R[0,0] = cov_x_x
        self.R[1,1] = cov_xd_xd
        self.R[2,2] = cov_y_y
        self.R[3,3] = cov_yd_yd
        self.R[4,4] = cov_rot_z_z
        assert(np.array_equal(self.R, np.diagflat(np.diag(self.R))))

        # prediction estimation - this is your "Priori"
        # robot starts from rest at (0, 0), pointing forward
        self.xhat = np.array([0, 0, 0, 0, 0]).reshape((KalmanFilter.N_STATES, 1))
        # covariance initial estimate
        # covariance is equal to R since initial estimate is based off /odom data
        self.P = self.R

    """Finds the most recent measurement from /imu given a time"""
    def _get_closest_input(self, t: float) -> dict:
        closest_input = None

        input_start_time = self._imu_data[0]['time']
        for input in self._imu_data:
            # compute elapsed time in seconds (convert from ns)
            elapsed_time = (input['time'] - input_start_time) / (1e9)
            # update closest input and break once we passed the desired time
            if elapsed_time <= t:
                closest_input = input
            else:
                break

        assert(closest_input is not None)  # should always find an input
        return closest_input

    """Convert orientation quaternion to robot heading"""
    def _orientation_to_heading(self, orientation: Quaternion) -> float:
        qx = orientation[0]
        qy = orientation[1]
        qz = orientation[2]
        qw = orientation[3]

        # convert quarternion to robot heading
        t3 = 2.0*(qw * qz + qx * qy)
        t4 = 1.0 - 2.0*(qy*qy + qz*qz)
        return math.atan2(t3, t4)

    """Convert quaternion to homogeneous transformation matrix"""
    def _tf_to_homogenous(self, translation: Vector3, rotation: Quaternion) -> np.array:
        qx = rotation[0]
        qy = rotation[1]
        qz = rotation[2]
        qw = rotation[3]

        # convert translation and quarternion to transformation matrix
        return np.array([
            [qw**2 + qx**2 - qy**2 - qz**2, 2*(qx*qy-qz*qw), translation[0]],
            [2*(qw*qz + qx*qy), qw**2 - qx**2 + qy**2 - qz**2, translation[1]],
            [0, 0, 1]
        ], np.float64)

    def run(self) -> States:
        n_data_points = len(self._odom_data)  # run filter as long as we have measurements

        # store timesteps and state for each iteration of Kalman filter
        self._T = self._dt*np.arange(0, n_data_points)
        self._state = np.zeros((n_data_points, KalmanFilter.N_STATES))

        t = 0
        for k in range(n_data_points):  # for each sensor measurement
            # get input
            closest_input = self._get_closest_input(t)
            a_x_robot = closest_input['linear_acceleration'][0]
            theta = self._orientation_to_heading(closest_input['orientation'])
            # decompose robot acceleration into global accelerations
            a_x = a_x_robot*np.cos(theta)
            a_y = a_x_robot*np.sin(theta)
            omega = closest_input['angular_velocity'][2]
            u = np.array([a_x, a_y, omega]).reshape((3, 1))

            # get measurement
            y_k = np.array([
                self._odom_data[k]['pose']['position'][0],
                self._odom_data[k]['twist']['linear'][0],
                self._odom_data[k]['pose']['position'][1],
                self._odom_data[k]['twist']['linear'][1],
                self._orientation_to_heading(self._odom_data[k]['pose']['orientation']),
            ]).reshape((KalmanFilter.N_STATES, 1))

            ## Kalman Filter Estimation ##
            # prediction update
            xhat_k = self.A@self.xhat + self.B@u  # no process noise on prediction
            P_predict = self.A@self.P@self.A.transpose() + self.Q
            # this co-variance is the prediction of essentially how the measurement
            # and sensor model move together in relation to each state and helps scale
            #  our kalman gain by giving the ratio. By Definition, P is the variance 
            # of the state space, and by applying it to the motion model we're getting
            # a motion uncertainty which can be propogated and applied to the 
            # measurement model and expand its uncertainty as well

            # measurement update and Kalman Gain
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

            # store estimated state
            self._state[k, :] = self.xhat.T

            t += self._dt # increment elapsed time

        return self._state

    """
    Calculate the actual pose of the robot from /tf
    This uses the transformation from base_footprint to odom
    """
    def _compute_true_robot_position(self) -> tuple[np.array, np.array]:
        tf_base_footprint_to_odom = self._tf_data[BASE_FOOTPRINT_TO_ODOM]
        # tf has 1 extra message than odom, drop it so plots are equal
        # this also avoids an error when calculating MSE
        tf_base_footprint_to_odom = tf_base_footprint_to_odom[0:len(self._odom_data)]
        # check tf and odom timestamps match
        # i.e. ground truth corresponds to correct measurements
        assert(len(tf_base_footprint_to_odom) == len(self._odom_data))
        assert(tf_base_footprint_to_odom[0]['time'] == self._odom_data[0]['time'])
        assert(tf_base_footprint_to_odom[-1]['time'] == self._odom_data[-1]['time'])

        n_points = len(tf_base_footprint_to_odom)  # number of measurements
        t = np.zeros((n_points, 1))
        robot_pose = np.zeros((n_points, 3))

        origin = np.array([0, 0, 1]).reshape((3, 1))  # homogenous coordinate of origin

        for i, tf in enumerate(tf_base_footprint_to_odom):
            # compute transform for current pose
            bf_to_odom = self._tf_to_homogenous(
                tf['transform']['translation'], tf['transform']['rotation']
            )
            # transform origin in base_footprint frame to odom frame
            bf_pose_in_odom = bf_to_odom@origin

            # replace homogenous coordinate with theta
            bf_pose_in_odom[2] = self._orientation_to_heading(tf['transform']['rotation'])
            # store robot pose in odom frame
            robot_pose[i] = bf_pose_in_odom.T
            # compute elapsed time for current pose
            t[i] = (tf['time'] - tf_base_footprint_to_odom[0]['time']) / (1e9)

        return (t, robot_pose)

    """Calculate Mean-Squared Error (MSE)"""
    def _mse(self, true, filter) -> float:
        return np.mean(np.square(filter - true))

    """Generate plots and report Mean-Squared Error (MSE)"""
    def results(self) -> tuple[float, float, float]:
        # calculate actual robot pose, used to calculate MSE
        t, true_robot_pose = self._compute_true_robot_position()

        # extract localized pose from state
        localized_pose = np.hstack([
            self._state[:, 0].reshape(-1, 1),
            self._state[:, 2].reshape(-1, 1),
            self._state[:, 4].reshape(-1, 1),
        ])

        """Plots vectors to show robot heading along the robot's path"""
        def plot_robot_heading(state) -> None:
            subsample_n: int = 75  # plot heading every n iterations of filter

            state_subsampled = state[::subsample_n]
            u = np.cos(state_subsampled[:, 2])
            v = np.sin(state_subsampled[:, 2])
            x = state_subsampled[:, 0]
            y = state_subsampled[:, 1]
            plt.quiver(x, y, u, v, angles='xy', color='tab:blue')

        # plot true planar position
        plt.figure()
        plt.scatter(true_robot_pose[:, 0], true_robot_pose[:, 1], c=t, s=1)
        plt.colorbar(label='Time (s)')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend(['True Position'])
        plot_robot_heading(true_robot_pose)

        # plot predicted planar position
        plt.figure()
        plt.scatter(localized_pose[:, 0], localized_pose[:, 1], c=self._T, s=1)
        plt.colorbar(label='Time (s)')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend(['Localized Position'])
        plot_robot_heading(localized_pose)

        # plot true and predicted planar position
        plt.figure()
        plt.plot(true_robot_pose[:, 0], true_robot_pose[:, 1])
        plt.plot(localized_pose[:, 0], localized_pose[:, 1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend(['True Position', 'Localized Position'])

        # plot each pose variable with respect to time
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

        ax1.plot(t, true_robot_pose[:, 0])
        ax1.plot(self._T, localized_pose[:, 0], linestyle='dashed')
        ax1.set_ylabel('x (m)')
        ax1.set_xlabel('Time (s)')

        ax2.plot(t, true_robot_pose[:, 1])
        ax2.plot(self._T, localized_pose[:, 1], linestyle='dashed')
        ax2.set_ylabel('y (m)')
        ax2.set_xlabel('Time (s)')

        ax3.plot(t, true_robot_pose[:, 2])
        ax3.plot(self._T, localized_pose[:, 2], linestyle='dashed')
        ax3.set_ylabel('Theta (rad)')
        ax3.set_xlabel('Time (s)')

        plt.figlegend(['True Position', 'Predicted Position'])
        fig.align_ylabels()

        # plot error of each pose variable with respect to time
        error = true_robot_pose - localized_pose
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

        ax1.plot(t, error[:, 0])
        ax1.set_ylabel('x Error (m)')
        ax1.set_xlabel('Time (s)')

        ax2.plot(t, error[:, 1])
        ax2.set_ylabel('y Error (m)')
        ax2.set_xlabel('Time (s)')

        ax3.plot(t, error[:, 2])
        ax3.set_ylabel('Theta Error (rad)')
        ax3.set_xlabel('Time (s)')

        plt.subplots_adjust(left=0.175)  # fixes an issue with ylabels being cutoff
        fig.align_ylabels()

        # calculate MSE
        x_mse = self._mse(true_robot_pose[:, 0], localized_pose[:, 0])
        y_mse = self._mse(true_robot_pose[:, 1], localized_pose[:, 1])
        theta_mse = self._mse(true_robot_pose[:, 2], localized_pose[:, 2])
        print('\n----- Mean Squared Error (MSE) -----')
        print('  X:', x_mse)
        print('  Y:', y_mse)
        print('  Theta:', theta_mse)

        plt.show()

        return (x_mse, y_mse, theta_mse) # return MSE for each pose variable


def main():
    print("Running Kalman filter...")
    kf = KalmanFilter('path.json')
    kf.run()
    kf.results()


if __name__ == '__main__':
    main()
