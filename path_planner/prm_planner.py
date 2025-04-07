import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from path_planner.utils import ObstaclesGrid
import random
from queue import PriorityQueue


class Node:
    def __init__(self, x, y):
        """
        Represents a node in the PRM roadmap.

        Args:
            x (float): X-coordinate of the node.
            y (float): Y-coordinate of the node.
        """
        self.x = x
        self.y = y

class PRMPlanner:
    def __init__(self, start, goal, map_size, obstacles, num_samples=200, k_neighbors=10, step_size=0.2):
        """
        Initializes the PRM planner.

        Args:
            start (tuple): (x, y) coordinates of the start position.
            goal (tuple): (x, y) coordinates of the goal position.
            map_size (tuple): (width, height) of the environment.
            obstacles (ObstaclesGrid): Object that stores obstacle information.
            num_samples (int): Number of random samples for roadmap construction.
            k_neighbors (int): Number of nearest neighbors to connect in the roadmap.
            step_size (float): Step size used for collision checking.
        """
        # self.start = Node(start[0]*10, start[1]*10)
        # self.goal = Node(goal[0]*10, goal[1]*10)
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.map_size = map_size
        self.obstacles = obstacles
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.step_size = step_size
        self.roadmap = [] 
        self.edges = {}

    def construct_roadmap(self):
        """
        Constructs the probabilistic roadmap by sampling nodes and connecting them.

        Returns:
        None
        """
        num_sample = self.num_samples
        node_list = []
        start = (self.start.x, self.start.y)
        goal = (self.goal.x, self.goal.y)
        node_list.append(start)
        node_list.append(goal)
        for i in range(num_sample-2):
            node = PRMPlanner.sample_free_point(self)
            while node in node_list:
                # if the node is existed, sample again
                node = PRMPlanner.sample_free_point(self)
            node_list.append(node)
        
        self.sample_node_list = node_list
        # find the k-neastest neighbor of nodes
        k = self.k_neighbors
        for i in range(len(node_list)):
            neighbor_list = PRMPlanner.find_k_nearest(self, node_list[i], k, node_list)
            for n in range(len(neighbor_list)):
                neighbor_node = neighbor_list[n]
                # find whether there is an edge between these two points or not
                if (node_list[i], neighbor_node) not in self.edges and (neighbor_node, node_list[i]) not in self.edges:
                    # determind the edge is collision or not
                    if PRMPlanner.is_colliding(self, node_list[i], neighbor_node):
                        continue
                    else:
                        # use the manhattan or other distance to compute the cost
                        cost = np.linalg.norm(np.array(node_list[i]) - np.array(neighbor_node))
                        self.edges[(node_list[i], neighbor_node)] = cost
        

    def sample_free_point(self):
        """
        Samples a random collision-free point in the environment.

        Returns:
        Node: A randomly sampled node.
        """
        width, height = self.map_size
        while True:
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            if self.obstacles.map[x, y] == False:
                break
        node = (x / 10, y / 10)
        # node = (x, y)
        return node
        '''
        n_row, n_col = self.map_size
        while True:
            row = random.randint(0, n_row - 1)
            col = random.randint(0, n_col - 1)
            if self.obstacles.map[row, col] == False:
                break
        node = (n_row, n_col)
        return node
        '''

    def find_k_nearest(self, node, k, sample_node_list):
        """
        Finds the k-nearest neighbors of a node in the roadmap.

        Args:
            node (Node): The node for which neighbors are searched.
            k (int): The number of nearest neighbors to find.

        Returns:
            list: A list of k-nearest neighbor nodes.
        """
        distance = dict()
        neighbor_list = []
        for i in range(len(sample_node_list)):
            d = np.array(sample_node_list[i]) - np.array(node)
            d = np.linalg.norm(d)
            distance[sample_node_list[i]] = d
        
        # sorted the distance
        distance = sorted(distance.items(), key= lambda x: x[1])
        for i in range(k):
            neighbor = distance[i][0]
            neighbor_list.append(neighbor)
        
        return neighbor_list

    def is_colliding(self, node1, node2):
        """
        Checks if the path between two nodes collides with an obstacle.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            bool: True if there is a collision, False otherwise.
        """
        # change the row col to x, y
        
        x_1 = node1[1] * 10
        y_1 = 100 - node1[0] * 10
        x_2 = node2[1] * 10
        y_2 = 100 - node2[0] * 10
        '''
        x_1 = node1[1]
        y_1 = 100 - node1[0]
        x_2 = node2[1]
        y_2 = 100 - node2[0]
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
                    print(node1, node2, (row, col))
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


    
    def create_adjacency_matrix(self, sample_node_list):
        # cost matrix store 
        edges = self.edges
        edges = list(edges.items())
        num = len(sample_node_list)
        cost_matrix = np.full((num, num), np.inf)
        for i in range(num):
            cost_matrix[i][i] = 0

        for i in range(len(edges)):
            node_1 = edges[i][0][0]
            node_2 = edges[i][0][1]
            cost = edges[i][1]
            idx_1 = sample_node_list.index(node_1)
            idx_2 = sample_node_list.index(node_2)
            ''''
            if idx_1 == 0  and idx_2 == 1:
                print(i, node_1, node_2)
                print(PRMPlanner.is_colliding(self, node_1, node_2))'
            '''
            if cost < cost_matrix[idx_1][idx_2] or cost < cost_matrix[idx_2][idx_1]:
                cost_matrix[idx_1][idx_2] = cost
                cost_matrix[idx_2][idx_1] = cost
        return cost_matrix
    
    def get_neighbor(self, node, costs_matrix, sample_node_list):
        idx = sample_node_list.index(node)
        neighbor_list = []  # store the neighbor nodes' index
        for i in range(len(costs_matrix)):
            if idx == i:
                continue
            else:
                if np.isinf(costs_matrix[idx][i]):
                    # this two points didn't connect, they are not neighbor
                    continue
                else:
                    neighbor_list.append(i)
        return neighbor_list
    
    def find_path(self, start, goal, parent_node):
        path = []
        child = goal
        path.append(child)
        while child != start:
            father = parent_node[child]
            path.append(father)
            child = father
        
        reverse_path = path[::-1]

        return reverse_path

    def plan(self):
        """
        Plans a path from start to goal using the constructed roadmap.

        Returns:
        list: A list of (x, y) tuples representing the path.
        """
        num_samples = self.num_samples
        sample_node_list = self.sample_node_list
        # create cost/adjacency matrix
        cost_matrix = PRMPlanner.create_adjacency_matrix(self, sample_node_list)
        # dijkstra algorithm
        start = (self.start.x, self.start.y)
        goal = (self.goal.x, self.goal.y)
        open_set = PriorityQueue()
        close_set = set()
        # dist stored the shorts cost of a node to start node.
        dist = dict()
        parent_node = dict()

        # put start points into priority queue
        open_set.put((0, start))
        dist[start] = 0
        
        # add start node's neighbor in the open set
        # begin the loop
        while not open_set.empty():
            _, point = open_set.get()
            if point in close_set:
                continue
            if point == goal:
                break
            close_set.add(point)

            neighbor_idx_list = PRMPlanner.get_neighbor(self, point, cost_matrix, sample_node_list)
            for idx in neighbor_idx_list:
                neighbor_node = sample_node_list[idx]
                point_idx = sample_node_list.index(point)
                if sample_node_list[idx] in close_set:
                    continue
                else:
                    #if sample_node_list[idx] in open_set:
                    if neighbor_node in dist:
                        cost = dist[point] + cost_matrix[point_idx][idx]
                        if cost < dist[neighbor_node]:
                            # update
                            dist[neighbor_node] = cost
                            open_set.put((cost, neighbor_node))
                            parent_node[neighbor_node] = point
                    else:
                        cost = cost_matrix[point_idx][idx]
                        dist[neighbor_node] = cost_matrix[point_idx][idx]
                        open_set.put((cost, neighbor_node))
                        parent_node[neighbor_node] = point
        
        path = PRMPlanner.find_path(self, start, goal, parent_node)

        print(path)

        # a* algorithm is similart with dijkstra algorithm.

        return path
