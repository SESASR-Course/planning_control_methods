from bfs import BFS
from queue import PriorityQueue


class Dijkstra(BFS):

    def __init__(self, grid_map, movements_class):
        super().__init__(grid_map, movements_class)
        self.frontier = PriorityQueue()
        self.costmap = self.generate_costmap()
        self.cost_so_far = {}

    def initialize_structures(self, start_node, goal_node):
        """Method to initialize the structures of the algorithm"""
        self.frontier.put(start_node, 0)
        self.came_from[start_node] = None
        self.cost_so_far[start_node] = 0

    def generate_costmap(self):
        """Method to generate the cost map"""
        costmap = self.grid_map.copy()
        # add a gaussian near the obstacles to avoid them
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                if self.grid_map[i, j] == 1:
                    costmap[i - 2 : i + 3, j - 2 : j + 3] += 50
        costmap[costmap == 1] = 1000
        return costmap

    def node_insertion(self, new_node, current, goal):
        """Method to insert a new node in the frontier"""
        new_node.g = self.cost_g(current, new_node)
        if new_node not in self.cost_so_far or new_node.g < self.cost_so_far[new_node]:
            self.cost_so_far[new_node] = new_node.g
            new_node.g = self.cost_g(current, new_node)
            self.frontier.put(new_node, new_node.g)
            self.came_from[new_node] = current

    def cost_g(self, current, neighbor):
        """Method to compute the cost from the start node to the current node"""
        return (
            current.g
            + self.movements_class.cost(current.position, neighbor.position)
            * self.costmap[tuple(neighbor.position.astype(int).tolist())]
        )
