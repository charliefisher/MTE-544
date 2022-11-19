from enum import IntEnum
import math
import warnings

import numpy as np
import matplotlib.pyplot as plt


class OccupancyStatus(IntEnum):
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 1


class TrinaryOccupancyMap:
    def __init__(self, csv_file: str, origin: tuple[float, float], cell_size: float) -> None:
        self._map = np.genfromtxt(csv_file, delimiter=',', dtype=np.int8)
        self._origin = origin
        self._cell_size = cell_size

    """Provides read-only access to underlying map data"""
    @property
    def map_data(self) -> np.array:
        return self._map

    """Converts world coordinates to indices in the map"""
    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        # map origin is defined at bottom left corner, but indexing starts at the top left corner
        # thus, the y coordinate has to be offset
        max_y_idx: int = self.map_data.shape[0] - 1
        x_idx = math.floor((x - self._origin[0])/self._cell_size)
        y_idx = max_y_idx - math.floor((y - self._origin[1])/self._cell_size)
        return (y_idx, x_idx)

    """Gets the occupancy status of a given coordinate"""
    def get_occupancy(self, x: float, y: float) -> bool:
        y_idx, x_idx = self.world_to_map(x, y)
        if x_idx < 0 or y_idx < 0 or x_idx >= self.map_data.shape[1] or y_idx >= self.map_data.shape[0]:
            warnings.warn("({}, {}) is outside of map!".format(x, y))
            return OccupancyStatus.UNKNOWN
        return self.map_data[y_idx, x_idx]

    """Checks if a given coordinate in world space is free"""
    def is_free(self, x: float, y: float) -> bool:
        return self.get_occupancy(x, y) == OccupancyStatus.FREE

    """Exports map data to KDTree format"""
    def to_kdtree(self) -> np.array:

        def map_to_world(coords: np.array) -> np.array:
            # map origin is defined at bottom left corner, but indexing starts at the top left corner
            # thus, the y coordinate has to be offset
            max_y_idx: int = self.map_data.shape[0] - 1
            x = self._origin[0] + self._cell_size*coords[:,1] + self._cell_size/2
            y = self._origin[1] + self._cell_size*(max_y_idx - coords[:,0]) + self._cell_size/2
            return np.array([x, y]).T

        occupied = np.argwhere(self.map_data == OccupancyStatus.OCCUPIED)
        return map_to_world(occupied)

    def plot(self) -> None:
        map_data = np.copy(self.map_data)
        map_data[map_data == OccupancyStatus.UNKNOWN] = np.iinfo(map_data.dtype).max / 1.15
        map_data[map_data == OccupancyStatus.FREE] = np.iinfo(map_data.dtype).max
        map_data[map_data == OccupancyStatus.OCCUPIED] = 0

        extent = (
            self._origin[0], self._origin[0] + map_data.shape[1]*self._cell_size,
            self._origin[1], self._origin[1] + map_data.shape[0]*self._cell_size
        )
        plt.imshow(map_data, cmap='gray', extent=extent)
        plt.xlim((extent[0], extent[1]))
        plt.ylim((extent[2], extent[3]))

    def plot_points(self, points: np.array) -> None:
        plt.scatter(points[0], points[1], c='r', s=0.2)

    def plot_particles(self, particles: np.array) -> None:
        for x, y, theta in particles:
            plt.scatter(x, y, c='b', s=10)
            arrow_len = 0.25
            plt.arrow(x, y, arrow_len*np.cos(theta), arrow_len*np.sin(theta), color='b')
