import utils
import matplotlib.pyplot as plt
from matplotlib.collections import CircleCollection
from utils import Node
import numpy as np


class Planner2D:
    """Base class to implement a 2D planner"""

    def __init__(self, grid_map, movements_class: utils.Movements, plot_enabled=False):
        self.grid_map = grid_map
        self.movements_class = movements_class
        self.frontier = (
            None  # PriorityQueue() # Queue() depending on the subclass implementation
        )
        self.came_from = {}
        self.plot_enabled = plot_enabled

        # Define the movements class to use in the planner
        # 8-connectivity movements
        # 4-connectivity movements

    def plan(self, start, goal, ax=None, fig=None):
        """Method to implement the Dijkstra algorithm

        Args:
          start: tuple, start position
          goal: tuple, goal position

        Returns:
          Path: list of tuples
        """

        def update_plot():
            """Update the plot with the current frame"""
            norm = plt.Normalize(vmin=0, vmax=255)
            nodes = list(self.came_from.keys())
            nodes = [node.position for node in nodes]
            nodes = np.array(nodes)
            nodes = np.vstack(nodes)
            node_costs = [node.f for node in list(self.came_from.keys())]
            node_costs = np.array(node_costs)
            nodes_x = nodes[:, 1]
            nodes_y = nodes[:, 0]
            nodes_for_plot = np.vstack(
                [nodes_x + 0.5, self.grid_map.shape[0] - nodes_y - 0.5]
            ).T
            scatter = CircleCollection(
                sizes=[30] * len(nodes),
                offsets=nodes_for_plot,
                array=norm(node_costs),
                cmap="viridis",
                edgecolors="face",
                transOffset=ax.transData,
                alpha=0.5,
            )

            return scatter

        start_node = Node(start)
        goal_node = Node(goal)

        # check if the start and goal nodes are valid
        if not (
            self.check_valid_position(start_node.position)
            and self.check_valid_position(goal_node.position)
        ):
            raise ValueError("Invalid start or goal node")
        if self.plot_enabled:
            self.plot_node(start_node, "green", ax)
            self.plot_node(goal_node, "red", ax)

        self.initialize_structures(start_node, goal_node)
        frames = []

        while self.frontier:
            frame = []
            current_node = self.frontier.get()

            if current_node == goal_node:  # Early exit condition
                break

            plot_to_update = False
            for (
                new_position
            ) in (
                self.movements_class.movements
            ):  # Explore the graph by visiting the neighbors
                node_position = new_position + current_node.position

                # Check if the new position is valid
                # (inside the map and not an obstacle)
                if not self.check_valid_position(node_position):
                    continue

                new_node = Node(
                    node_position
                )  # Create a new node with the current node as parent

                self.node_insertion(
                    new_node, current_node, goal_node
                )  # Insert the new node in the frontier
                # if at least one node was inserted in the frontier update the plot
                plot_to_update = True
            if plot_to_update and self.plot_enabled:
                scatter = update_plot()
                frame.append(ax.add_collection(scatter))
                frames.append(frame)
            else:
                print("No new nodes were inserted in the frontier")

        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = self.came_from[current]
            if self.plot_enabled:
                path_plt = self.plot_path(path, ax)
                frame = [ax.add_collection(scatter), path_plt]
                frames.append(frame)
            return path[::-1], frames

        raise ValueError("No path found")

    def node_insertion(self, new_node, current, goal):
        raise NotImplementedError("This method should be implemented in the subclass")

    def initialize_structures(self, start_node, goal_node):
        raise NotImplementedError("This method should be implemented in the subclass")

    def check_on_map(self, position):
        """Method to check if a position is inside the map"""
        if (
            position[0] > (self.grid_map.shape[0] - 1)
            or position[0] < 0
            or position[1] > (self.grid_map.shape[1] - 1)
            or position[1] < 0
        ):
            return False
        return True

    def check_obstacle(self, position):
        """Method to check if a position is an obstacle"""
        if self.grid_map[tuple(position.astype(int).tolist())] == 1:
            return True
        return False

    def check_valid_position(self, position):
        """Method to check if a position is valid"""
        return self.check_on_map(position) and not self.check_obstacle(position)

    def plot_grid_map(self):
        """Method to plot the grid map"""
        utils.plot_gridmap(self.grid_map, "Grid Map")

    def cost_g(self, current, neighbor):
        """Method to compute the cost from the start node to the current node"""
        raise NotImplementedError("This method should be implemented in the subclass")

    def cost_h(self, current, goal):
        """Method to compute the heuristic cost from the current node to the end node"""
        raise NotImplementedError("This method should be implemented in the subclass")

    def plot_node(self, node, color, ax, radius=0.3):
        """Method to plot a node in the grid map with a cross and color based on cost"""
        return ax.add_artist(
            plt.Circle(
                (
                    node.position[1] + 0.5,
                    self.grid_map.shape[0] - node.position[0] - 0.5,
                ),
                radius=radius,
                color=color,
            )
        )

    def plot_path(self, path, ax=None):
        """Method to plot the path found by the planner

        Args:
          path: list of tuples
        """
        return ax.add_artist(
            plt.Line2D(
                [x[1] + 0.5 for x in path],
                [self.grid_map.shape[0] - x[0] - 0.5 for x in path],
                color="blue",
                linewidth=2,
            )
        )
