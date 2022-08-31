from cmath import sqrt
from tokenize import Double
from matplotlib import pyplot as plt
import numpy as np

import math

class Node:
    def __init__(self, x, y, parent_node, index):
        self.x = x
        self.y = y

        self.parent_node = parent_node
        self.index = index
        
        self.cost = -1
    
    def distance(self, other):
        return math.sqrt( (self.x - other.x)**2 + (self.y - other.y) ** 2 )


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
        return self.Nodes[ int(self.get_node_index(x, y) )]

    def draw(self):
        """draws the grid with node indicies displayed in their corresponding (x, y) coordinates"""

        for node in self.Nodes:
            plt.text(node.x, node.y, str(node.index), color="red", \
            fontsize=8, horizontalalignment="center", verticalalignment = "center")  
        
        plt.xlim([self.min_x - self.spacing, self.max_x + self.spacing])
        plt.ylim([self.min_y - self.spacing, self.max_y + self.spacing])
        plt.xticks(ticks = [ x for x in np.arange(self.min_x, self.max_x + self.spacing, self.width / 5.0)])
        plt.yticks(ticks = [ y for y in np.arange(self.min_y, self.max_y + self.spacing, self.height / 5.0)])
        plt.show()

grid = Grid(0, 10, 0, 10, 0.5)
grid.draw()


node1 = Node(2, 1, None, 0)
node2 = Node(3, 2, None, 0)
print(node1.distance(node2))