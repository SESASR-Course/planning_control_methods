import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
from PIL import Image
import skimage
from typing import TypeVar


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
        return np.linalg.norm(new_pos - current_pos, np.inf)


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


def get_map(map_path, xy_reso, plot_map=False):
    """
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


def plot_gridmap(map, title, ax):
    cmap = colors.ListedColormap(["White", "Black"])

    ax.pcolor(map[::-1], cmap=cmap, edgecolors="k", linewidths=1)
    plot_map(map, title, ax)


def plot_costmap(costmap, title, ax):
    ax.pcolor(costmap[::-1], cmap="Blues", edgecolors="k", linewidths=1)

    norm = colors.Normalize(vmin=0, vmax=np.max(costmap))
    sm = plt.cm.ScalarMappable(cmap="Blues", norm=norm)
    sm.set_array([])

    plot_map(costmap, title, ax)


def plot_map(map, title, ax):
    if map.shape[0] < 20:
        ax.set_xticks(
            ticks=np.array(range(map.shape[1])) + 0.5, labels=range(map.shape[1])
        )
        ax.set_yticks(
            ticks=np.array(range(map.shape[0])) + 0.5,
            labels=range(map.shape[0] - 1, -1, -1),
        )
    else:
        ax.set_xticks(
            ticks=np.array(range(0, map.shape[1], 5)) + 0.5,
            labels=range(0, map.shape[1], 5),
        )
        ax.set_yticks(
            ticks=np.array(range(0, map.shape[0], 5)) + 0.5,
            labels=range(map.shape[0] - 1, -1, -5),
        )

    ax.set_xlim(0, map.shape[0])
    ax.set_ylim(0, map.shape[1])
    ax.set_title(title, fontsize=14)
