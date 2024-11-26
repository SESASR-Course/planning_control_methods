from planner_2d import Planner2D
from queue import PriorityQueue


class Greedy(Planner2D):

    def __init__(self, grid_map, movements_class):
        super().__init__(grid_map, movements_class)
        self.frontier = PriorityQueue()

    def initialize_structures(self, start_node, goal_node):
        """Method to initialize the structures of the algorithm"""
        self.frontier.put(start_node, 0)
        self.came_from[start_node] = None

    def node_insertion(self, new_node, current, goal):
        """Method to insert a new node in the frontier"""
        if new_node not in self.came_from:
            # new_node.g = self.cost_g(
            #     current, new_node
            # )  # cost from start node to current node is 0 for Greedy
            new_node.h = self.cost_h(new_node, goal)
            self.frontier.put(new_node, new_node.h)
            self.came_from[new_node] = current

    def cost_h(self, current, goal):
        """Method to compute the heuristic cost from the current node to the end node"""
        # print(self.movements_class.heuristic_cost(current.position, goal.position))
        return self.movements_class.heuristic_cost(current.position, goal.position)

    def cost_g(self, current, neighbor):
        """Method to compute the cost from the start node to the current node"""
        return 0
