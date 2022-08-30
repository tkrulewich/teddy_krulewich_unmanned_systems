from matplotlib import pyplot as plt
import numpy as np

import math

class Node:
    def __init__(self, x, y, parent_node, index):
        self.x = x
        self.y = y

        self.parent_node = parent_node
        self.index = index

        self.cost = math.inf
    
    def distance(self, other):
        return math.sqrt( (self.x - other.x)**2 + (self.y - other.y) ** 2 )
    
    def __lt__(self, other):
        return self.cost < other.cost


class Obstacle:
    def __init__(self, x: float, y: float, diameter: float):
        self.x = x
        self.y = y

        self.diameter = diameter
    
    def CollidesWith(self, node: Node):
        if (math.sqrt( (self.x - node.x)**2 + (self.y - node.y)**2) <= self.diameter / 2.0):
            return True
        
        return False
    
    def draw(self):
        circle = plt.Circle( (self.x, self.y), self.diameter / 2.0, color='blue')
        plt.gca().add_patch(circle)

class Grid:
    def __init__(self, min_x : float, max_x : float, min_y : float, max_y : float, spacing : float):
        self.min_x = min_x
        self.max_x = max_x

        self.min_y = min_y
        self.max_y = max_y


        self.width = max_x - min_x
        self.height = max_y - min_y

        self.spacing = spacing

        self.Nodes = []
        self.Obstacles = []

        index = 0
        for y in np.arange(min_y, max_y + spacing, spacing):
            for x in np.arange(min_x, max_x + spacing, spacing):
                self.Nodes.append(Node(x, y, None, index))
                index += 1
    
    def get_node_index(self, x: float, y : float):
        """Gets index of node closest to an x,y coordinate"""
        
        # if the node isn't on a grid point, use the closest x,y coordinate that is on the grid
        if (x % self.spacing != 0 or y % self.spacing != 0):

            # there are four possible closest points found by all combinations of
            # the x and y coordinates rounded up and down

            floor_x = x - x % self.spacing
            floor_y = y - y % self.spacing

            ceil_x = floor_x + self.spacing
            ceil_y = floor_y + self.spacing

            # the list of possible closest points
            neighbors = [ (floor_x, floor_y), (ceil_x, floor_y), (floor_x, ceil_y), (ceil_x, ceil_y) ]

            # find the closest
            min_dist = math.inf
            min_x, min_y = math.inf, math.inf
            for x2,y2 in neighbors:
                dist = math.sqrt( (x - x2)**2 +  (y - y2)**2 )
                if (dist < min_dist):
                    min_dist = dist
                    min_x = x2
                    min_y = y2
            
            # set x and y to the coordidnates of the closest point found
            x = min_x
            y = min_y
        
        # find the index of the point and return it
        index = (y - self.min_y) / self.spacing * (self.max_x / self.spacing + 1) + x / self.spacing
        return int(index)
    
    
    def get_node(self, x, y):
        """Gets the node instance closest to a given x,y coodinate"""

        if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
            return None
        
        return self.Nodes[ int(self.get_node_index(x, y) )]

    def draw(self):
        """draws the grid with node indicies displayed in their corresponding (x, y) coordinates"""

        for obstacle in self.Obstacles:
            obstacle.draw()
        
        for node in self.Nodes:
            plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        
        plt.xlim([self.min_x - self.spacing, self.max_x + self.spacing])
        plt.ylim([self.min_y - self.spacing, self.max_y + self.spacing])
        plt.xticks(ticks = [ x for x in np.arange(self.min_x, self.max_x + self.spacing, self.width / 5.0)])
        plt.yticks(ticks = [ y for y in np.arange(self.min_y, self.max_y + self.spacing, self.height / 5.0)])
    
    def add_obstacle(self, obstacle: Obstacle):
        self.Obstacles.append(obstacle)
    
    def add_obstacles(self, obstacles: list):
        self.Obstacles.extend(obstacles)

    def djikstras(self, start, end):

        start.cost = 0

        current_node : Node = start

        unvisited = set(self.Nodes)
        visited = set()

        while len(unvisited) > 0:
            unvisited.remove(current_node)
            visited.add(current_node)
            
            for y in np.arange(current_node.y - self.spacing, current_node.y + 2 * self.spacing, self.spacing):
                for x in np.arange(current_node.x - self.spacing, current_node.x + 2 * self.spacing, self.spacing):
                    
                    neighbor = self.get_node(x, y)

                    if (self.node_valid(neighbor) and neighbor is not current_node):
                        cost = current_node.cost + current_node.distance(neighbor)
                        if (cost < neighbor.cost):
                            neighbor.cost = cost
                            neighbor.parent_node = current_node
            
            if len(unvisited) > 0:
                current_node = min(unvisited, key=lambda x: x.cost)
            
        x_list = []
        y_list = []

        current_node = end
        while current_node != None:
            x_list.append(current_node.x)
            y_list.append(current_node.y)

            current_node = current_node.parent_node
        
        return x_list, y_list
    
    def node_valid(self, node: Node):
        if node is None:
            return False

        for obstacle in self.Obstacles:
            if obstacle.CollidesWith(node):
                return False
        
        return True
        


grid = Grid(0, 10, 0, 10, 0.5)

grid.add_obstacles([
    Obstacle(5, 5, 5), 
    Obstacle(3, 4, 1.5),
    Obstacle(5, 0, 1.5),
    Obstacle(5, 1, 2.5),
    Obstacle(0, 7, 0.5),
    Obstacle(1, 7, 0.5),
    Obstacle(2, 7, 0.5),
    Obstacle(3, 7, 1.5)])


path = grid.djikstras( grid.get_node(0, 0), grid.get_node(8,9))
grid.draw()

for node in path:
    x, y = path

    plt.plot(x, y)

plt.show()

node = grid.get_node_index(8, 4.5)

print(node)