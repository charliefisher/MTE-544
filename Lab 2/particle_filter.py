import math
import warnings

import numpy as np


MAP_ORIGIN: tuple[float, float] = (-1.94, -8.63)
MAP_CELL_SIZE: float = 0.03


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
    def world_to_map(self, x: float, y: float) -> tuple[float, float]:
        # map origin is defined at bottom left corner, but indexing starts at the top left corner
        # thus, the y coordinate has to be offset
        max_y_idx: int = self._map.shape[0] - 1
        x_idx = math.floor((x - self._origin[0])/self._cell_size)
        y_idx = max_y_idx - math.floor((y - self._origin[1])/self._cell_size)
        return (y_idx, x_idx)

    """Checks if a given coordinate in world space is occupied"""
    def is_occupied(self, x: float, y: float) -> bool:
        y_idx, x_idx = self.world_to_map(x, y)
        if x_idx < 0 or y_idx < 0 or x_idx >= self._map.shape[1] or y_idx >= self._map.shape[0]:
            warnings.warn("({}, {}) is outside of map!".format(x, y))
            return True
        return bool(self._map[y_idx, x_idx])  # convert np.bool to python bool


class ParticleFilter:
    def __init__(self) -> None:
        self._map = BinaryOccupancyMap('binary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)

    def run(self) -> tuple[float, float]:
        pass


def test():
    map = BinaryOccupancyMap('binary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)
    assert not map.is_occupied(6, -2)
    assert map.is_occupied(-0.0650001, -2.075)
    assert map.is_occupied(-0.0650001, -2.105)
    assert map.is_occupied(-0.0350001, -2.105)
    assert map.binary_data[239, 63]
    print('Passed Tests!')


if __name__ == '__main__':
    pf = ParticleFilter()
    pf.run()
    