import math
import warnings

import numpy as np
import matplotlib.pyplot as plt


class BinaryOccupancyMap:
    def __init__(self, csv_file: str, origin: tuple[float, float], cell_size: float) -> None:
        self._map = np.genfromtxt(csv_file, delimiter=',', dtype=np.uint8).astype(bool)
        self._origin = origin
        self._cell_size = cell_size

    """Provides access to underlying map data"""
    @property
    def binary_data(self) -> np.array:
        return self._map

    """Converts world coordinates to indices in the map"""
    def world_to_map(self, x: float, y: float) -> tuple[int, int]:
        # map origin is defined at bottom left corner, but indexing starts at the top left corner
        # thus, the y coordinate has to be offset
        max_y_idx: int = self._map.shape[0] - 1
        x_idx = math.floor((x - self._origin[0])/self._cell_size)
        y_idx = max_y_idx - math.floor((y - self._origin[1])/self._cell_size)
        return (y_idx, x_idx)

    def map_to_world(self, x_idx: int, y_idx: int) -> tuple[float, float]:
        # map origin is defined at bottom left corner, but indexing starts at the top left corner
        # thus, the y coordinate has to be offset
        max_y_idx: int = self._map.shape[0] - 1
        x = self._origin[0] + self._cell_size*x_idx + self._cell_size/2
        y = self._origin[1] + self._cell_size*(max_y_idx - y_idx) + self._cell_size/2
        return (x, y)

    """Checks if a given coordinate in world space is occupied"""
    def is_occupied(self, x: float, y: float) -> bool:
        y_idx, x_idx = self.world_to_map(x, y)
        if x_idx < 0 or y_idx < 0 or x_idx >= self._map.shape[1] or y_idx >= self._map.shape[0]:
            warnings.warn("({}, {}) is outside of map!".format(x, y))
            return False
        return bool(self._map[y_idx, x_idx])  # convert np.bool to python bool

    def _plot_map(self) -> None:
        map_size = self._map.shape
        extent = (
            self._origin[0], self._origin[0] + map_size[1]*self._cell_size,
            self._origin[1], self._origin[1] + map_size[0]*self._cell_size
        )
        plt.imshow(np.logical_not(self._map), cmap='gray', extent=extent)
        plt.xlim((extent[0], extent[1]))
        plt.ylim((extent[2], extent[3]))

    def plot_points(self, points: np.array) -> None:
        self._plot_map()
        plt.scatter(points[:,0], points[:,1], c='r', s=0.2)

    def plot_particles(self, particles: np.array) -> None:
        self._plot_map()
        for x, y, theta in particles:
            plt.scatter(x, y, c='b', s=10)
            arrow_len = 0.25
            plt.arrow(x, y, arrow_len*np.cos(theta), arrow_len*np.sin(theta), color='b')
