import numpy as np
import matplotlib.pyplot as plt


class node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __eq__(self, other):
        return self.position == other.position


diagonal_distance = int(2**(0.5))
orthogonal_distance = 1.0
occupancy_threshold = 70
occupancy_scaling_factor = 0.15


def astar(maze, start, end, greediness):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end node
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal, you can also implement what should happen if there is no possible path
        if current_node == end_node:
            # Complete here code to return the shortest path found
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        # Complete here code to generate children, which are the neighboring nodes. You should use 4 or 8 points connectivity for a grid.
        children = []
        for new_position in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
            node_position = (
                current_node.position[0]+new_position[0], current_node.position[1]+new_position[1])

            if node_position[0] > (len(maze)-1) or node_position[0] < 0 or node_position[1] > (len(maze[0])-1) or node_position[1] < 0:
                continue
            if maze[node_position[0]][node_position[1]] >= occupancy_threshold:
                continue

            children.append(node(current_node, node_position))

        # Loop through children to update the costs
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    break
            else:
                # Create the f, g, and h values, replace the 0s with appropriate formulations of the costs
                if (child.position[0] - current_node.position[0] == 0 or child.position[1] - current_node.position[1] == 0):
                    child.g = current_node.g + orthogonal_distance + \
                        maze[child.position[0]][child.position[1]] * \
                        occupancy_scaling_factor
                else:
                    child.g = current_node.g + diagonal_distance + maze[child.position[0]][child.position[1]] * \
                        occupancy_scaling_factor
                child.h = ((child.position[0] - end_node.position[0])
                           ** 2 + (child.position[1] - end_node.position[1])**2)
                child.f = child.g + greediness*child.h

                # Complete here code to check whether to add a child to the open list
                for open_node in open_list:
                    if child == open_node and child.f > open_node.f:
                        break
                else:
                    if child in open_list:
                        open_list.remove(child)
                    open_list.append(child)
    return []


def main():

    # Load your maze here
    maze = []

    # This is an example maze you can use for testing, replace it with loading the actual map
    # maze = [[0,   0,   0,   0,   1,   0, 0, 0, 0, 0],
    # [0, 0.8,   1,   0,   1,   0, 0, 0, 0, 0],
    # [0, 0.9,   1,   0,   1,   0, 1, 0, 0, 0],
    # [0,   1,   0,   0,   1,   0, 1, 0, 0, 0],
    # [0,   1,   0,   0,   1,   0, 0, 0, 0, 0],
    # [0,   0,   0, 0.9,   0,   1, 0, 0, 0, 0],
    # [0,   0, 0.9,   1,   1, 0.7, 0, 0, 0, 0],
    # [0,   0,   0,   1,   0,   0, 0, 0, 0, 0],
    # [0,   0,   0,   0, 0.9,   0, 0, 0, 0, 0],
    # [0,   0,   0,   0,   0,   0, 0, 0, 0, 0]]
    #maze = np.zeros((192,312))

    maze = np.genfromtxt(
        "/home/soyazhekhan/Documents/lab3/src/path_planner_node/path_planner_node/costmap(1).csv", delimiter=',', dtype=np.int8)

    maze = np.transpose(maze)
    # maze = np.flip(maze,axis=0)
    # maze = np.flip(maze,axis=1)

    # Define here your start and end points
    start = (5, 15)
    end = (22, 22)
    greediness = 1

    # Compute the path with your implementation of Astar
    path = np.asarray(astar(maze, start, end, greediness), dtype=np.float)
    maze_plot = np.transpose(np.nonzero(maze))
    # print("shape {}".format(maze_plot.shape()))s

    plt.plot(maze_plot[:, 0], maze_plot[:, 1], '.')

    if not np.any(path):  # If path is empty, will be NaN, check if path is NaN
        print("No path found")
    else:
        plt.plot(path[:, 0], path[:, 1])
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
