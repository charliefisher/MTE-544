from occupancy_map import BinaryOccupancyMap


MAP_ORIGIN: tuple[float, float] = (-1.94, -8.63)
MAP_CELL_SIZE: float = 0.03


class ParticleFilter:
    def __init__(self) -> None:
        self._map = BinaryOccupancyMap('binary_occupancy_map.csv', MAP_ORIGIN, MAP_CELL_SIZE)

    def run(self) -> tuple[float, float]:
        pass


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
    pf = ParticleFilter()
    pf.run()
    