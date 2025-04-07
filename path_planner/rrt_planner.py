import numpy as np
import matplotlib.pyplot as plt
from path_planner.utils import ObstaclesGrid
import random

class Node:
    def __init__(self, x, y, parent=None):
        """
        Represents a node in the RRT tree.
        
        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
            parent (Node, optional): Parent node in the tree.
        """
        self.x = x
        self.y = y
        self.parent = parent  

class RRTPlanner:
    def __init__(self, start, goal, map_size, obstacles, max_iter=500, step_size=0.3):
        """
        Initializes the RRT planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            max_iter (int): Maximum number of iterations for RRT.
            step_size (float): Step size for expanding the tree.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        # self.start = Node(start[0] * 10, start[1] * 10)
        # self.goal = Node(goal[0] * 10, goal[1] * 10)
        self.map_size = map_size
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.tree = [self.start] 
        # self.tree = [(self.start.x, self.start.y)]

    def plan(self):
        """
        Implements the RRT algorithm to find a path from start to goal.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """
        for i in range(self.max_iter):
            rand_node = self.sample_random_point()  
            nearest_node = self.find_nearest_node(rand_node)  
            new_node = self.steer(nearest_node, rand_node)

            if new_node and not self.is_colliding(new_node, nearest_node): 
                self.tree.append(new_node)

                if self.reached_goal(new_node):  
                    return self.construct_path(new_node) 
        
        print("Path not found.")
        return None

    def sample_random_point(self):
        """
        Samples a random point in the map.
        
        Returns:
            Node: A randomly sampled node.
        """
        random_porb = random.random()
        if random_porb > 0.5:
            width, height = self.map_size
            while True:
                x = random.randint(0, width - 1)
                y = random.randint(0, height - 1)
                if self.obstacles.map[x, y] == False:
                    break
            # because lattice size is * 10, so row, col need to /10
            node = Node(x / 10, y / 10)
            # node = Node(x, y)
        else:
            node = self.goal
        return node
    

    def find_nearest_node(self, rand_node):
        """
        Finds the nearest node in the tree to a given random node.

        Args:
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: The nearest node in the tree.
        """
        tree = self.tree
        min_distance = float('inf')
        for i in range(len(tree)):
            d = np.array([tree[i].x, tree[i].y]) - np.array([rand_node.x, rand_node.y])
            distance = np.linalg.norm(d)
            if distance < min_distance:
                nearest_node = tree[i]
                min_distance = distance
        
        return nearest_node


    def steer(self, nearest_node, rand_node):
        """
        Generates a new node by moving from the nearest node toward the random node.

        Args:
            nearest_node (Node): The nearest node in the tree.
            rand_node (Node): The randomly sampled node.

        Returns:
            Node: A new node in the direction of rand_node.
        """
        step_size = self.step_size
        d = np.array([nearest_node.x, nearest_node.y])-np.array([rand_node.x, rand_node.y])
        distance = np.linalg.norm(d)
        scale = step_size / distance
        node_x = nearest_node.x + scale * (rand_node.x - nearest_node.x)
        node_y = nearest_node.y + scale * (rand_node.y - nearest_node.y)
        node = Node(node_x, node_y)
        node.parent = nearest_node
        return node

    def is_colliding(self, new_node, nearest_node):
        """
        Checks if the path between nearest_node and new_node collides with an obstacle.

        Args:
            new_node (Node): The new node to check.
            nearest_node (Node): The nearest node in the tree.

        Returns:
            bool: True if there is a collision, False otherwise.
        """
        
        x_1 = new_node.y * 10
        y_1 = 100 - new_node.x * 10
        x_2 = nearest_node.y * 10
        y_2 = 100 - nearest_node.x * 10
        '''
        x_1 = new_node.y
        y_1 = 100 - new_node.x
        x_2 = nearest_node.y
        y_2 = 100 - nearest_node.x
        '''
        if x_1 <= x_2:
            start = (int(x_1), int(y_1))
            end = (int(x_2), int(y_2))
        else:
            start = (int(x_2), int(y_2))
            end = (int(x_1), int(y_1))
        
        # line function
        if end[0] != start[0]:
            m = (end[1] - start[1]) / (end[0] - start[0])
            b = start[1] - m * start[0]
            for x in range(start[0], end[0]):
                y = m * x + b
                row = 100 - int(y)
                col = int(x)
                if row >= 100 or col >= 100:
                    print(new_node, nearest_node, (row, col))
                if self.obstacles.map[row, col] == True:
                    return True
        else:
            # it means the robot just go down or go up
            up_y  = max(start[1], end[1])
            down_y = min(start[1], end[1])
            for y in range(down_y, up_y+1):
                col = int(start[0])
                row = 100 - int(y)
                if self.obstacles.map[row, col] == True:
                    return True
        return False
 
    def reached_goal(self, new_node):
        """
        Checks if the goal has been reached.

        Args:
            new_node (Node): The most recently added node.

        Returns:
            bool: True if goal is reached, False otherwise.
        """
        
        if new_node.x == self.goal.x and new_node.y == self.goal.y:
            self.goal.parent = new_node.parent
            return True
        else:
            d = np.array([new_node.x, new_node.y]) - np.array([self.goal.x, self.goal.y])
            distance = np.linalg.norm(d)
            if distance <= self.step_size:   # use the step size as threshold
                self.goal.parent = new_node
                return True
            else:
                return False   

    def construct_path(self, end_node):
        """
        Constructs the final path by backtracking from the goal node to the start node.

        Args:
            end_node (Node): The node at the goal position.

        Returns:
            list: A list of (x, y) tuples representing the path from start to goal.
        """
        path = []
        if end_node.x != self.goal.x and end_node.y != self.goal.y:
            path.append((self.goal.x, self.goal.y))
        child_node = end_node
        path.append((child_node.x, child_node.y))
        while child_node.x != self.start.x and child_node.y != self.start.y:
            parent_node = child_node.parent
            path.append((parent_node.x, parent_node.y))
            child_node = parent_node
        # path.append((se.x, child_node.y))

        reverse_path = path[::-1]
        print(reverse_path)
        return reverse_path
