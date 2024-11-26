import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from PIL import Image
import skimage
from typing import TypeVar


def get_map(map_path, xy_reso, plot_map=False):
    """ "
    Load the image of the 2D map and convert into numpy ndarray with xy resolution
    Return:
      - full map: np array
      - gridmap resized: np array
    """
    img = Image.open(map_path)
    npmap = np.asarray(img, dtype=int)
    print("Map size:", npmap.shape)

    if plot_map:
        plot_gridmap(npmap, "Full Map")

    # reduce the resolution: from the original map to the grid map using max pooling
    grid_map = skimage.measure.block_reduce(npmap, (xy_reso, xy_reso), np.max)
    print("Grid Map size:", grid_map.shape)

    return npmap, grid_map


def calc_grid_map_config(map_size, xyreso):
    minx = 0
    miny = 0
    maxx = map_size[0]
    maxy = map_size[1]
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return xw, yw


def compute_map_occ(map):
    """
    Compute occupancy state for each cell of the gridmap
    Possible states:
      - occupied = 1 (obstacle present)
      - free = 0
      - unknown = not defined (usually -1)
    Returns two np arrays with poses of the obstacles in the map and all the map poses.
    It supports the pre-computation of likelihood field over the entire map
    """
    n_o = np.count_nonzero(map)
    obst_poses = np.zeros((n_o, 2), dtype=int)
    end_points = np.zeros((map.shape[0] * map.shape[1], 2), dtype=int)

    i = 0
    j = 0
    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            if map[x, y] == 1:
                obst_poses[i, :] = x, y
                i += 1

            end_points[j, :] = x, y
            j += 1

    return obst_poses, end_points


def plot_gridmap(map, title):
    cmap = colors.ListedColormap(["White", "Black"])
    plt.figure(figsize=(6, 6))
    plt.pcolor(map[::-1], cmap=cmap, edgecolors="k", linewidths=1)
    if map.shape[0] < 20:
        plt.xticks(ticks=range(map.shape[1] + 1), labels=range(map.shape[1] + 1))
        plt.yticks(ticks=range(map.shape[0]), labels=range(map.shape[0], 0, -1))
    else:
        plt.xticks(
            ticks=range(0, map.shape[1] + 1, 5), labels=range(0, map.shape[1] + 1, 5)
        )
        plt.yticks(ticks=range(0, map.shape[0], 5), labels=range(map.shape[0], 0, -5))
    plt.axis("equal")
    plt.title(title, fontsize=14)


def plot_costmap(costmap, title):
    plt.figure(figsize=(6, 6))
    plt.pcolor(costmap[::-1], cmap="viridis", edgecolors="k", linewidths=1)
    plt.colorbar()
    if costmap.shape[0] < 20:
        plt.xticks(
            ticks=range(costmap.shape[1] + 1), labels=range(costmap.shape[1] + 1)
        )
        plt.yticks(ticks=range(costmap.shape[0]), labels=range(costmap.shape[0], 0, -1))
    else:
        plt.xticks(
            ticks=range(0, costmap.shape[1] + 1, 5),
            labels=range(0, costmap.shape[1] + 1, 5),
        )
        plt.yticks(
            ticks=range(0, costmap.shape[0], 5), labels=range(costmap.shape[0], 0, -5)
        )
    plt.axis("equal")
    plt.title(title, fontsize=14)


class Movements:
    """Base class to implement the movements"""

    def __init__(self):
        self._movements = None

    @property
    def movements(self):
        """Method to get the movements"""
        return self._movements.tolist()

    def cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the cost of the movement"""
        movement = new_pos - current_pos
        if movement in self._movements:
            return np.linalg.norm(movement, 2)
        raise ValueError(f"Invalid movement for the movement class: {type(self)}")

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        raise NotImplementedError("This method should be implemented in the subclass")


class Movements8Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array(
            [
                (1.0, 0.0),
                (1.0, 1.0),
                (0.0, 1.0),
                (-1.0, 1.0),
                (-1.0, 0.0),
                (-1.0, -1.0),
                (0.0, -1.0),
                (1.0, -1.0),
            ]
        )

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, 2)


class Movements4Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array([(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)])

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, 1)


SelfTypeNode = TypeVar("SelfTypeNode", bound="Node")


class Node:
    """Class to represent the nodes of the graph"""

    def __init__(self, position: np.ndarray):
        self.position = position

        self.g = 0  # cost from start node to current node
        self.h = 0  # heuristic cost from current node to end node

    @property
    def f(self):
        return self.g + self.h

    # The following methods are used to compare the nodes

    def __eq__(self, other):  # equal
        return np.array_equal(self.position, other.position)

    def __lt__(self, other):  # less than
        return self.f < other.f

    def __le__(self, other):  # less or equal
        return self.f <= other.f

    def __repr__(self):  # return a string representation of the object
        return f"Node {self.position} with cost {self.f}"

    def __hash__(self):  # required for instances to be usable as keys in hash tables
        return hash(tuple(self.position))
