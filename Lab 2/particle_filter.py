import json

import numpy as np
import matplotlib.pyplot as plt

from occupancy_map import BinaryOccupancyMap


MAP_ORIGIN: tuple[float, float] = (-1.94, -8.63)
MAP_CELL_SIZE: float = 0.03


Pose = np.array # 3 x 1 vector
Poses = np.array # n x 3 matrix


class ParticleFilter:
    def __init__(self, n_particles: float) -> None:
        self._n_particles = n_particles
        self._map = BinaryOccupancyMap('binary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)
        self._angle_min: float = None
        self._angle_max: float = None
        self._angle_increment: float = None
        self._range_min: float = None
        self._range_max: float = None

    def _lidar_scan_to_cartesian(self, scan_data: np.array) -> np.array:
        scan_data = np.clip(scan_data, self._range_min, self._range_max)
        angle = np.arange(self._angle_min, self._angle_max + self._angle_increment, self._angle_increment)
        cartesian = (np.array([np.cos(angle), np.sin(angle)])*scan_data).T
        cartesian_len = cartesian.shape[0]
        return np.append(cartesian, np.ones((cartesian_len, 1)), axis=1)

    def _generate_inital_particles(self) -> Poses:
        particles = []
        while len(particles) < self._n_particles:
            x = np.random.uniform(-1.94, 8.74)
            y = np.random.uniform(-8.63, 5.08)

            if self._map.is_occupied(x, y):
                continue

            theta = np.random.uniform(-np.pi, np.pi)
            particles.append(np.array([x, y, theta]))

        return np.array(particles)

    def _lidar_frame_to_robot(self, scan_data_cartesian: np.array) -> np.array:
        rot_ang = -np.pi/2
        T_robot_lidar = np.array([
            [np.cos(rot_ang), -np.sin(rot_ang), -0.04],
            [np.sin(rot_ang), np.cos(rot_ang), 0],
            [0, 0, 1]
        ], np.float64)
        return scan_data_cartesian@T_robot_lidar

    def _robot_frame_to_map(self, robot_pose: Pose, scan_data_robot: np.array) -> np.array:
        rot_ang = robot_pose[2]
        T_map_robot = np.array([
            [np.cos(rot_ang), -np.sin(rot_ang), robot_pose[0]],
            [np.sin(rot_ang), np.cos(rot_ang), robot_pose[1]],
            [0, 0, 1]
        ], np.float64)
        return scan_data_robot@T_map_robot

    def _weight_particle(self) -> float:
        pass

    def _run(self, scan_data: np.array) -> Pose:
        scan_data_cartesian = self._lidar_scan_to_cartesian(scan_data)

        scan_data_robot = self._lidar_frame_to_robot(scan_data_cartesian)
        assert np.all(np.not_equal(scan_data_cartesian, scan_data_robot))  # check transformation succeeded
        
        robot_poses = self._generate_inital_particles()
        print('robot_poses', robot_poses.shape)

        all_points = np.ndarray((len(scan_data)*self._n_particles, 2))
        for i, pose in enumerate(robot_poses):
            scan_data_map = self._robot_frame_to_map(pose, scan_data_robot)
            print('scan_data_map', scan_data_map.shape)
            assert np.all(np.not_equal(scan_data_robot, scan_data_map))  # check transformation succeeded
            all_points[i*len(scan_data):(i+1)*len(scan_data)] = scan_data_map[:,0:2]
            
            print('robot pose is', pose)
            plt.figure()
            self._map.plot_particles([pose])
            # print('extracted pts', scan_data_map[:,0:2].shape)
            self._map.plot_points(scan_data_map[:,0:2])
            plt.show()

        print('all_points', all_points.shape)
        self._map.plot_points(all_points)



    def localize(self, lidar_data_file: str) -> tuple[float, float]:
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
        assert all(len(scan_data[0])== len(s) for s in scan_data)

        # run filter where a single iteration is a single record of scan data
        for scan in scan_data:
            lidar_measurement = np.array(scan)
            self._run(lidar_measurement)
            break # TODO: remove me, just for testing


def test():
    map = BinaryOccupancyMap('binary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)

    # check specific known locations
    assert not map.is_occupied(6, -2)
    assert map.is_occupied(-0.0650001, -2.075)
    assert map.is_occupied(-0.0650001, -2.105)
    assert map.is_occupied(-0.0350001, -2.105)

    # check underlying data of known location
    assert map.binary_data[239, 63]

    # check out of bounds access
    assert map.is_occupied(-2, -9)
    assert map.is_occupied(10, 6)

    print('Passed Tests!')


if __name__ == '__main__':
    pf = ParticleFilter(50)
    pf.localize('point2.json')
    