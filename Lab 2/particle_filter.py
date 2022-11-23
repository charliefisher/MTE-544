import os
import json
import shutil
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

from occupancy_map import TrinaryOccupancyMap


FIGURE_DIR: str = 'figures'

MAP_ORIGIN: tuple[float, float] = (-1.94, -8.63)
MAP_LIMIT: tuple[float, float] = (8.74, 5.08)
MAP_CELL_SIZE: float = 0.03


Pose = np.array  # 3 x 1 vector
Poses = np.array  # n x 3 matrix


class ParticleFilter:
    def __init__(self, n_particles: float) -> None:
        self._n_particles = n_particles
        self._map = TrinaryOccupancyMap(
            'trinary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)
        self._kdt = KDTree(self._map.to_kdtree())
        self._data_name: Optional[str] = None

        self._angle_max: float = None
        self._angle_increment: float = None
        self._range_min: float = None
        self._range_max: float = None

        self._robot_poses = None
        self._weights = np.ndarray(self._n_particles)

        self._last_best_pose = None
        self._last_best_pose_pts = None

    def _remove_bad_measurements(self, scan_data: np.array) -> tuple[np.array, np.array]:
        # take only samples where lidar distance is finite
        valid_data = np.isfinite(scan_data)
        angle = np.arange(self._angle_min, self._angle_max +
                          self._angle_increment, self._angle_increment)
        scan_data = scan_data[valid_data]
        angle = angle[valid_data]

        # clip scan data to correct range
        scan_data = np.clip(scan_data, self._range_min, self._range_max)
        assert np.all(np.logical_and(np.less_equal(
            self._range_min, scan_data), np.less_equal(scan_data, self._range_max)))

        return (angle, scan_data)

    def _lidar_scan_to_cartesian(self, angle: np.array, scan_data: np.array) -> np.array:
        # convert lidar polar coordinates to cartesian (in lidar frame)
        cartesian = (np.array([np.cos(angle), np.sin(angle)])*scan_data)
        cartesian_len = cartesian.shape[1]
        return np.append(cartesian, np.ones((1, cartesian_len)), axis=0)

    def _generate_inital_particles(self) -> Poses:
        particles = []
        while len(particles) < self._n_particles:
            x = np.random.uniform(MAP_ORIGIN[0], MAP_LIMIT[0])
            y = np.random.uniform(MAP_ORIGIN[1], MAP_LIMIT[1])

            if not self._map.is_free(x, y):
                continue

            theta = np.random.uniform(-np.pi, np.pi)
            particles.append(np.array([x, y, theta]))

        return np.array(particles).T

    def _lidar_frame_to_robot(self, scan_data_cartesian: np.array) -> np.array:
        rot_ang = np.pi/2
        T_robot_lidar = np.array([
            [np.cos(rot_ang), -np.sin(rot_ang), -0.04],
            [np.sin(rot_ang), np.cos(rot_ang), 0],
            [0, 0, 1]
        ], np.float64)
        return T_robot_lidar@scan_data_cartesian

    def _robot_frame_to_map(self, robot_pose: Pose,
                            scan_data_robot: np.array) -> np.array:
        theta = robot_pose[2]
        T_map_robot = np.array([
            [np.cos(theta), -np.sin(theta), robot_pose[0]],
            [np.sin(theta), np.cos(theta), robot_pose[1]],
            [0, 0, 1]
        ], np.float64)
        return T_map_robot@scan_data_robot

    def _weight_particle(self, scan_data_map: np.array, lidar_std: float) -> float:
        distances = self._kdt.query(scan_data_map[0:2].T, k=1)[0][:]
        # TODO: try sum
        return np.prod(np.exp(-(distances**2)/(2*lidar_std**2)))

    def _get_best_pose(self) -> Pose:
        best_pose_idx = np.argmax(self._weights)
        return self._robot_poses[:, best_pose_idx]

    def _variance_particles(self) -> float:
        return np.var(self._robot_poses, axis=1)

    def _mse_best_particle(self, scan_data_cartesian: np.array) -> float:
        best_pose = self._get_best_pose()
        scan_data_robot = self._lidar_frame_to_robot(scan_data_cartesian)
        best_particle_lidar_pts = self._robot_frame_to_map(
            best_pose, scan_data_robot)[0:2]

        distances = self._kdt.query(best_particle_lidar_pts[0:2].T, k=1)[0][:]
        return np.sum(np.square(distances))

    def _run(self, iter: int, scan_data: np.array) -> Pose:
        assert self._robot_poses is not None

        if iter <= 10 and self._data_name is not None:  # plot if specified
            plt.figure()
            self._map.plot()
            self._map.plot_particles(self._robot_poses.T)
            plt.savefig(
                '{}/{}_i{}.png'.format(FIGURE_DIR, self._data_name, iter))
            plt.close()

        (angle, scan_data) = self._remove_bad_measurements(scan_data)
        scan_data_cartesian = self._lidar_scan_to_cartesian(angle, scan_data)

        if iter % 10 == 0:
            print('Iteration: {}'.format(iter))
            print('   Variance:', self._variance_particles())
            print('   MSE:', self._mse_best_particle(scan_data_cartesian))

        lidar_std = np.std(scan_data)

        scan_data_robot = self._lidar_frame_to_robot(scan_data_cartesian)

        if iter > 0:
            # add noise to points (except on first iteration which are random guesses)
            sigma = np.std(self._robot_poses, axis=1, keepdims=1)
            # sigma = np.exp(-1/20*iter)*np.array([[0.25], [0.25], [0.5]])
            # print('sigma', sigma)
            noise = sigma * np.random.randn(3, self._n_particles)
            self._robot_poses = self._robot_poses + noise

        for i, pose in enumerate(self._robot_poses.T):
            # lidar data in map frame
            scan_data_map = self._robot_frame_to_map(pose, scan_data_robot)
            # compute error of pose
            self._weights[i] = self._weight_particle(scan_data_map, lidar_std)

        # normalize weights
        self._weights = self._weights / np.max(self._weights)
        self._weights = self._weights/self._weights.sum(axis=0, keepdims=1)

        # resample points
        resample = np.random.choice(
            np.arange(0, self._n_particles), p=self._weights, size=self._n_particles)
        self._robot_poses = self._robot_poses[:, resample]

        best_pose = self._get_best_pose()
        self._last_best_pose = best_pose
        self._last_best_pose_pts = self._robot_frame_to_map(
            best_pose, scan_data_robot)[0:2]

    def localize(self, lidar_data_file: str,
                 data_name: Optional[str] = None) -> tuple[float, float]:
        # read json file
        with open(lidar_data_file, 'r') as data_file:
            data = json.load(data_file)

        # set parameters from sensor
        self._angle_min = data['angle_min']
        self._angle_max = data['angle_max']
        self._angle_increment = data['angle_increment']
        self._range_min = data['range_min']
        self._range_max = data['range_max']

        scan_data = data['scan_data']

        # check each lidar scan has the same number of measurements
        assert all(len(scan_data[0]) == len(s) for s in scan_data)

        self._data_name = data_name
        print('Starting Particle Filter...')

        # randomize particles for first iteration
        self._robot_poses = self._generate_inital_particles()

        # run filter where a single iteration is a single record of scan data
        for i, scan in enumerate(scan_data):
            lidar_measurement = np.array(scan)
            self._run(i, lidar_measurement)

        # get result of filter
        robot_pose = self._get_best_pose()
        print('\nRobot Position:', robot_pose, )

        # plot robot in localized position with lidar data
        final_lidar_measurement = np.array(scan_data[-1])
        (angle, scan_data) = self._remove_bad_measurements(final_lidar_measurement)
        scan_data_cartesian = self._lidar_scan_to_cartesian(angle, scan_data)
        scan_data_robot = self._lidar_frame_to_robot(scan_data_cartesian)
        robot_lidar_pts = self._robot_frame_to_map(
            robot_pose, scan_data_robot)[0:2]

        print('  Variance:', self._variance_particles())
        print('  MSE:', self._mse_best_particle(scan_data_cartesian))
        print()

        assert np.all(np.equal(self._last_best_pose, robot_pose))
        assert np.all(np.equal(self._last_best_pose_pts, robot_lidar_pts))

        plt.figure()
        self._map.plot()
        self._map.plot_particle(robot_pose.T)
        self._map.plot_points(robot_lidar_pts)
        if self._data_name is not None:
            plt.savefig('{}/{}_final.png'.format(FIGURE_DIR, self._data_name))
        plt.show()


def test():
    from occupancy_map import OccupancyStatus

    map = TrinaryOccupancyMap(
        'trinary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)

    # check specific known locations
    assert map.get_occupancy(6, -2) == OccupancyStatus.UNKNOWN
    assert map.get_occupancy(-0.0650001, -2.075) == OccupancyStatus.OCCUPIED
    assert map.get_occupancy(-0.0650001, -2.105) == OccupancyStatus.OCCUPIED
    assert map.get_occupancy(-0.0350001, -2.105) == OccupancyStatus.OCCUPIED

    # check underlying data of known location
    assert map.map_data[239, 63]

    # check out of bounds access
    assert map.get_occupancy(-2, -9) == OccupancyStatus.UNKNOWN
    assert map.get_occupancy(10, 6) == OccupancyStatus.UNKNOWN

    print('Passed Tests!')


if __name__ == '__main__':
    if os.path.exists(FIGURE_DIR):
        shutil.rmtree(FIGURE_DIR)
    os.mkdir(FIGURE_DIR)

    pf = ParticleFilter(10000)
    pf.localize('point2.json', 'p2')
    pf.localize('point4.json', 'p4')
