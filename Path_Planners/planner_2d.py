import utils
import matplotlib.pyplot as plt
from utils import Node


class Planner2D:
    """Base class to implement a 2D planner"""

    def __init__(self, grid_map, movements_class: utils.Movements):
        self.grid_map = grid_map
        self.movements_class = movements_class
        self.frontier = (
            None  # PriorityQueue() # Queue() depending on the subclass implementation
        )
        self.came_from = {}

        # Define the movements class to use in the planner
        # 8-connectivity movements
        # 4-connectivity movements

    def plan(self, start, goal):
        """Method to implement the Dijkstra algorithm

        Args:
          start: tuple, start position
          goal: tuple, goal position

        Returns:
          Path: list of tuples
        """

        start_node = Node(start)
        goal_node = Node(goal)

        # check if the start and goal nodes are valid
        if not (
            self.check_valid_position(start_node.position)
            and self.check_valid_position(goal_node.position)
        ):
            raise ValueError("Invalid start or goal node")

        self.plot_start(start_node)
        self.plot_goal(goal_node)

        self.initialize_structures(start_node, goal_node)

        while self.frontier:
            current_node = self.frontier.get()

            if current_node == goal_node:  # Early exit condition
                break

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

                self.plot_node(current_node, plt.cm.viridis)

        if current_node == goal_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = self.came_from[current]
            return path[::-1]

        raise ValueError("No path found")

    def node_insertion(self, new_node, current, goal):
        raise NotImplementedError("This method should be implemented in the subclass")

    def initialize_structures(self, start_node, goal_node):
        raise NotImplementedError("This method should be implemented in the subclass")

    def plot_path(self, path):
        """Method to plot the path found by the planner

        Args:
          path: list of tuples
        """

        plt.plot(
            [x[1] + 0.5 for x in path],
            [self.grid_map.shape[0] - x[0] - 0.5 for x in path],
            color="red",
            linewidth=2,
        )

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

    def plot_node(self, node, color_map):
        """Method to plot a node in the grid map with a cross and color based on cost"""
        norm = plt.Normalize(vmin=0, vmax=self.grid_map.shape[0] * 1.5)
        plt.plot(
            node.position[1] + 0.5,
            self.grid_map.shape[0] - node.position[0] - 0.5,
            color=color_map(norm(node.f)),
            marker="x",
        )

    def plot_start(self, start_node):
        plt.plot(
            start_node.position[1] + 0.5,
            self.grid_map.shape[0] - start_node.position[0] - 0.5,
            "go",
        )

    def plot_goal(self, goal_node):
        plt.plot(
            goal_node.position[1] + 0.5,
            self.grid_map.shape[0] - goal_node.position[0] - 0.5,
            "ro",
        )
