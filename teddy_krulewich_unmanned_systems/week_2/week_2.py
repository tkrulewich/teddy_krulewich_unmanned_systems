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
    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y

        self.radius = radius
    
    def collides_with(self, node: Node) -> bool:
        if (math.sqrt( (self.x - node.x)**2 + (self.y - node.y)**2) <= self.radius):
            return True
        
        return False
    
    def draw(self) -> None:
        circle = plt.Circle( (self.x, self.y), self.radius, color='blue')
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

        self.nodes : dict[tuple[float, float], Node] = {}
        self.valid_nodes : set[Node] = set()

        self.obstacles : list[Obstacle] = []

        index = 0
        for y in np.arange(min_y, max_y + spacing, spacing):
            for x in np.arange(min_x, max_x + spacing, spacing):
                node = Node(x, y, None, index)
                self.nodes[(x,y)] = node
                
                self.valid_nodes.add(node)
                index += 1    
    
    def get_node(self, x, y) -> Node:
        """Gets the node instance closest to a given x,y coodinate"""
        return self.nodes[(x, y)]

    def draw(self) -> None:
        """draws the grid with node indicies displayed in their corresponding (x, y) coordinates"""

        for obstacle in self.obstacles:
            obstacle.draw()
        
        x_list = [node.x for node in self.valid_nodes if math.isfinite(node.cost)]
        y_list = [node.y for node in self.valid_nodes if math.isfinite(node.cost)]

        #plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        plt.plot(x_list, y_list, marker = '.', linestyle='none', color='blue', markersize=0.25)

        x_list2 = [node.x for node in self.valid_nodes if math.isinf(node.cost)]
        y_list2 = [node.y for node in self.valid_nodes if math.isinf(node.cost)]

        #plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        plt.plot(x_list2, y_list2, marker = '.', linestyle='none', color='red', markersize=1)



        plt.xlim([self.min_x - self.spacing, self.max_x + self.spacing])
        plt.ylim([self.min_y - self.spacing, self.max_y + self.spacing])
        plt.xticks(ticks = [ x for x in np.arange(self.min_x, self.max_x + self.spacing, self.width / 5.0)])
        plt.yticks(ticks = [ y for y in np.arange(self.min_y, self.max_y + self.spacing, self.height / 5.0)])
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        self.obstacles.append(obstacle)


        # remove nodes from c-space that collide with object
        obs_x =  self.spacing * round(obstacle.x / self.spacing)
        obs_y =  self.spacing * round(obstacle.y / self.spacing)
        obs_r = self.spacing * round(obstacle.radius / self.spacing) + self.spacing;

        for y in np.arange(obs_y - obs_r, obs_y + obs_r, self.spacing):
            for x in np.arange(obs_x - obs_r, obs_x + obs_r, self.spacing):
                if (x, y) in self.nodes:
                    node = self.nodes[(x, y)]
                    if (node in self.valid_nodes and obstacle.collides_with(node)):
                        self.valid_nodes.remove(node)

    def add_obstacles(self, obstacles: list) -> None:
        for obstacle in obstacles:
            self.add_obstacle(obstacle)


    def dijkstras(self, start, end) -> tuple[list[float], list[float]]:
        start.cost = 0

        current_node : Node = start

        unvisited = set(self.nodes.values())
        seen = set([current_node])
        visited = set()

        while len(unvisited) > 0:
            unvisited.remove(current_node)
            visited.add(current_node)
            
            for y in np.arange(current_node.y - self.spacing, current_node.y + 2 * self.spacing, self.spacing):
                for x in np.arange(current_node.x - self.spacing, current_node.x + 2 * self.spacing, self.spacing):
                    if ((x, y) in self.nodes):
                        neighbor = self.nodes[(x,y)]

                        if (neighbor is not current_node and neighbor not in visited and neighbor in self.valid_nodes):
                            seen.add(neighbor)

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

grid = Grid(0, 10, 0, 10, 0.5)

grid.add_obstacles([
    Obstacle(1,1, 0.25),
    Obstacle(4,4, 0.25),
    Obstacle(3,4, 0.25),
    Obstacle(5,0, 0.25),
    Obstacle(5,1, 0.25),
    Obstacle(0,7, 0.25),
    Obstacle(1,7, 0.25),
    Obstacle(2,7, 0.25),
    Obstacle(3,7, 0.25)
])


fig = plt.figure(1)


start = grid.nodes[(0,0)]
end = grid.nodes[(8, 9)]

path = grid.dijkstras(start, end) 

x, y = path
x.reverse()
y.reverse()

path_line, = plt.plot(x, y, color='red')

# def update(frame):
#     path_line.set_data(x[:frame], y[:frame])
#     return path_line,

# anim = FuncAnimation(fig, update, frames=range(0, len(x)+1),blit=True, interval=50)


grid.draw()
plt.show()