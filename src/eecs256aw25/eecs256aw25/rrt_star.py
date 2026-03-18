import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self,
                 start: tuple = (0, 0),
                 goal: tuple = (0, 0),
                 obstacle_grid: np.array = None,
                 grid_resolution: float = 0.25,
                 expand_distance: float = 5.0,
                 connect_circle_dist: float = 20.0):

        self.start = Node(start[0], start[1])           # Start Node in the grid
        self.goal = Node(goal[0], goal[1])              # End Node in the grid
        self.grid = obstacle_grid                       # The obstacle grid to parse
        self.expand_dist = expand_distance              # Maximum distance to grow
        self.connect_circle_dist = connect_circle_dist  # 
        self.node_list = [self.start]                   # Graph will definitely contain the start node
        self.grid_resolution = grid_resolution          # Resolution of the obstacle grid

    def plan(self, N:int = 100):
        """
        Generate an RRT* plan after running N iterations.
        """

        # Generate graph for N iterations
        for iteration in range(N):
            rand_node = self.get_random_node()
            nearest_idx = self.get_nearest_node_idx(rand_node)
            nearest_node = self.node_list[nearest_idx]
            
            new_node = self.steer(nearest_node, rand_node)
            
            if self.check_collision(nearest_node, new_node):
                near_inds = self.get_near_nodes(new_node)
                new_node = self.connect(new_node, near_inds)
                self.node_list.append(new_node)
                self.rewire_graph(new_node, near_inds)

        # Generate final path
        return

    def steer(self, start_node: Node = None, end_node: Node = None, extend_length=float("inf")) -> Node:
        """
        Create a node in the direction of the "end" node from the "start" node.
        """
        dist, angle = self.get_distance_and_angle(start_node, end_node)
        step_len = min(extend_length, self.expand_dist, dist)
        
        new_node = Node(start_node.x + step_len * math.cos(angle),
                        start_node.y + step_len * math.sin(angle))
        new_node.cost = start_node.cost + step_len
        new_node.parent = start_node
        return new_node

    def connect(self, new_node, near_inds):
        """
        Connect the tree along a minimum cost path.
        """
        min_cost = new_node.cost
        min_parent = new_node.parent

        for i in near_inds:
            near_node = self.node_list[i]
            if self.check_collision(near_node, new_node):
                cost = near_node.cost + self.get_distance(near_node, new_node)
                if cost < min_cost:
                    min_cost = cost
                    min_parent = near_node

        new_node.parent = min_parent
        new_node.cost = min_cost
        return new_node

    def rewire_graph(self, new_node, near_inds):
        """
        Rewire the graph.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            if self.check_collision(new_node, near_node):
                new_cost = new_node.cost + self.get_distance(new_node, near_node)
                if new_cost < near_node.cost:
                    near_node.parent = new_node
                    near_node.cost = new_cost
                    self._propagate_cost_to_leaves(near_node)

    def _propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + self.get_distance(parent_node, node)
                self._propagate_cost_to_leaves(node)

    def check_collision(self, n_1: Node = None, n_2: Node = None) -> bool:
        """
        Use Bresenham's Line algorithm to check for obstacles.
        """
        x0, y0 = int(n_1.x), int(n_1.y)
        x1, y1 = int(n_2.x), int(n_2.y)
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            # Check boundaries
            if y0 < 0 or y0 >= self.grid.shape[0] or x0 < 0 or x0 >= self.grid.shape[1]:
                return False
            # Check obstacle
            if self.grid[y0, x0] == 1:
                return False
            
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True

    def get_random_node(self):
        """
        Get a random position from the provided grid.
        """
        x = random.uniform(0, self.grid.shape[1] - 1)
        y = random.uniform(0, self.grid.shape[0] - 1)
        return Node(x, y)

    def get_distance(self, n_1: Node = None, n_2: Node = None) -> float:
        """
        Return euclidean distance between 2 nodes.
        """
        return math.hypot(n_2.x - n_1.x, n_2.y - n_1.y)

    def get_distance_and_angle(self, n_1: Node = None, n_2: Node = None) -> tuple:
        """
        Compute the euclidean distance and the angle between the 2 nodes.
        """
        dx = n_2.x - n_1.x
        dy = n_2.y - n_1.y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        return dist, angle

    def get_distance_list(self, n: Node = None) -> list:
        """
        Get distance between the provided Node "n" and all the nodes in the graph.
        """
        return [self.get_distance(n, node) for node in self.node_list]

    def get_near_nodes(self, new_node: Node = None) -> list:
        """
        Get a list of the near nodes in the current graph for a given node.
        """
        dist_list = self.get_distance_list(new_node)
        return [i for i, d in enumerate(dist_list) if d <= self.connect_circle_dist]

    def get_nearest_node_idx(self, random_node: Node = None) -> int:
        """
        Get the index of the nearest node to the "random" node.
        """
        dist_list = self.get_distance_list(random_node)
        return dist_list.index(min(dist_list))

    def plot(self, visualize_nodes: bool = False) -> None:
        """
        Plot the graph.
        """

        plt.figure(figsize=(10, 10))

        # Display obstacle grid
        width = self.grid.shape[1] * self.grid_resolution
        height = self.grid.shape[0] * self.grid_resolution
        plt.imshow(self.grid, cmap='Greys', origin='lower', 
                extent=[0, width, 0, height])

        # Hightlight Start Node
        plt.scatter([self.start.x * self.grid_resolution],
                   [self.start.y * self.grid_resolution], c="green", s=100, alpha=1.0)

        # Hightlight Start Node
        plt.scatter([self.goal.x * self.grid_resolution],
                   [self.goal.y * self.grid_resolution], c="red", s=100, alpha=1.0)

        # Plot all edges in the graph
        if visualize_nodes:
            for node in self.node_list:
                if node.parent is not None:
                    plt.plot([node.x * self.grid_resolution,
                             node.parent.x * self.grid_resolution],
                             [node.y * self.grid_resolution,
                              node.parent.y * self.grid_resolution], 
                            color="blue", alpha=0.5)

            # Plot the nodes
            plt.scatter([n.x * self.grid_resolution for n in self.node_list],
                       [n.y * self.grid_resolution for n in self.node_list], 
                        c="red", s=10, alpha=0.5)

        # Set the ticks for the principal axes to denote the grid
        plt.xticks(np.arange(0, width, self.grid_resolution), minor=True)
        plt.yticks(np.arange(0, height, self.grid_resolution), minor=True)
        plt.grid(which='minor', alpha=0.1)

        plt.title(f"RRT* Graph: {len(self.node_list)} Nodes")
        plt.xlabel("X")
        plt.ylabel("Y")

        plt.show()

if __name__ == "__main__":

    # Load Grid
    grid = np.loadtxt("grid_1.txt", delimiter=",")
    # grid = np.loadtxt("grid_2.txt", delimiter=",")

    # Create RRT* object
    rrt_star = RRTStar(start=(5, 5),
                   goal=(110, 110),
                   obstacle_grid=grid,
                   expand_distance=3,
                   connect_circle_dist=5)
    # Plan
    rrt_star.plan(100)

    # Plot
    rrt_star.plot(visualize_nodes=True)