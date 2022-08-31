from turtle import distance
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np

import math

class Node:
    def __init__(self, x: float, y: float, parent_node, index : int):
        self.x = x
        self.y = y

        self.parent_node = parent_node
        self.index = index

        self.cost = math.inf
    
    def distance(self, other) -> float:
        return math.sqrt( (self.x - other.x)**2 + (self.y - other.y) ** 2 )
    
    def __lt__(self, other) -> bool:
        return self.cost < other.cost


class Obstacle:
    def __init__(self, x: float, y: float, diameter: float):
        self.x = x
        self.y = y

        self.diameter = diameter
    
    def collides_with(self, node: Node) -> bool:
        if (math.sqrt( (self.x - node.x)**2 + (self.y - node.y)**2) <= self.diameter / 2.0):
            return True
        
        return False
    
    def draw(self) -> None:
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

        self.Nodes : dict[tuple[float, float], Node] = {}
        self.Obstacles : list[Obstacle] = []

        index = 0
        for y in np.arange(min_y, max_y + spacing, spacing):
            for x in np.arange(min_x, max_x + spacing, spacing):
                self.Nodes[(x,y)] = Node(x, y, None, index)
                index += 1    
    
    def get_node(self, x, y) -> Node:
        """Gets the node instance closest to a given x,y coodinate"""
        return self.Nodes[(x, y)]

    def draw(self) -> None:
        """draws the grid with node indicies displayed in their corresponding (x, y) coordinates"""

        for obstacle in self.Obstacles:
            obstacle.draw()
        
        x_list = [node.x for node in self.Nodes.values()]
        y_list = [node.y for node in self.Nodes.values()]


        #plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        plt.plot(x_list, y_list, marker = '.', linestyle='none', markersize=0.5)

        plt.xlim([self.min_x - self.spacing, self.max_x + self.spacing])
        plt.ylim([self.min_y - self.spacing, self.max_y + self.spacing])
        plt.xticks(ticks = [ x for x in np.arange(self.min_x, self.max_x + self.spacing, self.width / 5.0)])
        plt.yticks(ticks = [ y for y in np.arange(self.min_y, self.max_y + self.spacing, self.height / 5.0)])
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        self.Obstacles.append(obstacle)



        for y in np.arange(obstacle.y - obstacle.diameter * 2.0, obstacle.y + obstacle.diameter, self.spacing):
            for x in np.arange(obstacle.x - obstacle.diameter * 2.0, obstacle.x + obstacle.diameter, self.spacing):
                if (x, y) in self.Nodes:
                    if (obstacle.collides_with(self.Nodes[(x,y)])):
                        del self.Nodes[(x, y)]

    def add_obstacles(self, obstacles: list) -> None:
        for obstacle in obstacles:
            self.add_obstacle(obstacle)


    def djikstras(self, start, end) -> tuple[list[float], list[float]]:
        start.cost = 0

        current_node : Node = start

        unvisited = set(self.Nodes.values())
        seen = set([current_node])
        visited = set()

        while len(unvisited) > 0:
            unvisited.remove(current_node)
            visited.add(current_node)
            
            for y in np.arange(current_node.y - self.spacing, current_node.y + 2 * self.spacing, self.spacing):
                for x in np.arange(current_node.x - self.spacing, current_node.x + 2 * self.spacing, self.spacing):
                    if ((x, y) in self.Nodes):
                        neighbor = self.Nodes[(x,y)]

                        if (neighbor not in visited):
                            seen.add(neighbor)

                        if (neighbor is not current_node and neighbor not in visited):
                            cost = current_node.cost + current_node.distance(neighbor)
                            if (cost < neighbor.cost):
                                neighbor.cost = cost
                                neighbor.parent_node = current_node
            
            if len(unvisited) > 0:
                seen.remove(current_node)
                if (len(seen) == 0):
                    break
                current_node = min(seen, key=lambda x: x.cost)
            
        x_list = []
        y_list = []

        current_node = end
        while current_node != None:
            x_list.append(current_node.x)
            y_list.append(current_node.y)

            current_node = current_node.parent_node
        
        plt.text(start.x, start.y, str(round(start.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        plt.text(end.x, end.y, str(round(end.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")

        return x_list, y_list
        

import random

grid = Grid(0, 100, 0, 100, 0.5)

for i in range(0, 50):
    x = random.randint(0, 100)
    y = random.randint(0, 100)

    d = random.randint(1, 20) * 0.5

    grid.add_obstacle(Obstacle(x, y, d))


start = random.choice(list(grid.Nodes.values()))
end = random.choice(list(grid.Nodes.values()))
ln, = plt.plot([], [], color='red')
grid.draw()

path = grid.djikstras( start, end) 

x, y = path
x.reverse()
y.reverse()
fig = plt.figure(1)

def init():
    return ln,

def update(frame):
    ln.set_data(x[:frame], y[:frame])
    return ln,



ani = FuncAnimation(fig, update, init_func=init,frames=range(0, len(x)+1),blit=True, interval=50)
grid.draw()
plt.show()