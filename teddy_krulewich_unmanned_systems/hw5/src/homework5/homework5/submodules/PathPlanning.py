from fileinput import close
from functools import reduce
from queue import PriorityQueue
from turtle import distance
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

import numpy as np
import math
import random

class Node:
    def __init__(self, x: float, y: float, parent_node, index : int):
        self.x = x
        self.y = y

        self.visited = False
        self.parent_node = parent_node
        self.index = index

        self.cost = math.inf
        self.heruistic = 0
    
    def distance(self, other) -> float:
        return math.sqrt( (self.x - other.x)**2 + (self.y - other.y) ** 2 )
    
    def __lt__(self, other) -> bool:
        return self.cost < other.cost


class BoundingBox:
    """Bounding box represents a region of space.
    It us used to reduce the search space for collision checking.
    Rather than check every node in the grid for collision, we can
    check only the nodes that are within the bounding box of an obstacle.
    """
    def __init__(self, min_x, max_x, min_y, max_y):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

        self.width = max_x - min_x
        self.height = max_y - min_y

        self.inflation_radius = 0

    def contains_node(self, node) -> bool:
        """ returns true if the node is within the bounding box accounting for the shrink factor"""
        return (self.min_x - self.inflation_radius <= node.x <= self.max_x  + self.inflation_radius
            and self.min_y  - self.inflation_radius <= node.y <= self.max_y + self.inflation_radius)
    
    def inflate(self, inflation_radius):
        """Inflates the bounding box by the given radius"""
        self.inflation_radius = inflation_radius

class Obstacle:
    def __init__(self, x: float, y: float, radius: float):
        self.x = x
        self.y = y

        self.radius = radius
        self.inflation_radius = 0
    
    def collides_with(self, node: Node) -> bool:
        """Returns true if a node clooides with this obstacle"""

        if (math.sqrt( (self.x - node.x)**2 + (self.y - node.y)**2) < self.radius + self.inflation_radius):
            return True
        
        return False

    def inflate(self, inflation_radius):
        """inflates the obstacle by the given radius"""
        self.inflation_radius = inflation_radius
    

    def get_bounding_box(self) -> BoundingBox:
        """Returns a bounding box that contains the obstacle including its inflation radius"""
        bounds = BoundingBox(self.x - self.radius, self.x + self.radius, self.y - self.radius, self.y + self.radius)
        bounds.inflate(self.inflation_radius)

        return bounds

    def draw(self) -> None:
        """Draws the obstacle to the screen with a transparent circle for the infalted radius"""
        obstacle_circle = plt.Circle( (self.x, self.y), self.radius, color='blue')
        plt.gca().add_patch(obstacle_circle)
        if (self.inflation_radius > 0):
            inflated_circle = plt.Circle( (self.x, self.y), self.radius + self.inflation_radius, color='red', alpha=0.5)
            plt.gca().add_patch(inflated_circle)

class Grid:
    def __init__(self, min_x : float, max_x : float, min_y : float, max_y : float, spacing : float):
        self.bounds = BoundingBox(min_x, max_x, min_y, max_y)

        self.spacing = spacing

        self.nodes : dict[tuple[float, float], Node] = {}
        self.valid_nodes : set[Node] = set()

        self.obstacles : list[Obstacle] = []

        self.inflation_radius = 0

        index = 0
        for y in np.arange(min_y, max_y + spacing, spacing):
            for x in np.arange(min_x, max_x + spacing, spacing):
                node = Node(x, y, None, index)
                self.nodes[(x,y)] = node
                
                self.valid_nodes.add(node)
                index += 1    
    
    def get_node(self, x, y) -> Node:
        """Gets the node instance closest to a given x,y coodinate"""

        if (x,y) in self.nodes:
            return self.nodes[(x,y)]
        

        x = round(x / self.spacing) * self.spacing
        y = round(y / self.spacing) * self.spacing

        if (x < self.bounds.min_x):
            x = self.bounds.min_x
        elif (x > self.bounds.max_x):
            x = self.bounds.max_x

        if (y < self.bounds.min_y):
            y = self.bounds.min_y
        elif (y > self.bounds.max_y):
            y = self.bounds.max_y

        return self.nodes[(x, y)]

    def draw(self, show_obstacles = True) -> None:
        """draws the grid with node indicies displayed in their corresponding (x, y) coordinates"""

        if show_obstacles:
            # draw all the obstacles
            for obstacle in self.obstacles:
                obstacle.draw()
            
        # get x and y coordinates of all the nodes that are valid
        x_list = [node.x for node in self.valid_nodes if math.isfinite(node.cost) and self.valid_nodes]
        y_list = [node.y for node in self.valid_nodes if math.isfinite(node.cost) and self.valid_nodes]

        #plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        
        # draw the valid nodes
        plt.plot(x_list, y_list, marker = '.', linestyle='none', color='blue', markersize=0.25)

        # get x and y coordinates of all the nodes that are invalid
        x_list2 = [node.x for node in self.valid_nodes if math.isinf(node.cost)]
        y_list2 = [node.y for node in self.valid_nodes if math.isinf(node.cost)]

        x_list3 = [node.x for node in self.valid_nodes if node.visited]
        y_list3 = [node.y for node in self.valid_nodes if node.visited]



        #plt.text(node.x, node.y, str(round(node.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        
        # draw the invalid nodes
        plt.plot(x_list2, y_list2, marker = '.', linestyle='none', color='red', markersize=1)
        plt.plot(x_list3, y_list3, marker = '.', linestyle='none', color='green', markersize=1)


        # set the ticks and limits based on spacing
        plt.xlim([self.bounds.min_x - self.spacing, self.bounds.max_x + self.spacing])
        plt.ylim([self.bounds.min_y - self.spacing, self.bounds.max_y + self.spacing])
        plt.xticks(ticks = [ x for x in np.arange(self.bounds.min_x, self.bounds.max_x + self.spacing, self.bounds.width / 5.0)])
        plt.yticks(ticks = [ y for y in np.arange(self.bounds.min_y, self.bounds.max_y + self.spacing, self.bounds.height / 5.0)])
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Adds an obstacle to the grid and invalidates all nodes that collide with it"""
        self.obstacles.append(obstacle)
        self.invalidate_neighboring_nodes(obstacle)

    def invalidate_neighboring_nodes(self, obstacle: Obstacle):
        """Invalidates all nodes that collide with the given obstacle"""

        # get the obstalce bounding box
        bounds = obstacle.get_bounding_box()

        # round the obtacle bounding box to the grid spacing
        bounds.min_x = self.spacing * math.floor(bounds.min_x / self.spacing)
        bounds.max_x = self.spacing * math.ceil(bounds.max_x / self.spacing)

        bounds.min_y = self.spacing * math.floor(bounds.min_y / self.spacing)
        bounds.max_y = self.spacing * math.ceil(bounds.max_y / self.spacing)

        # invalidate all the nodes that are within the bounding box
        for y in np.arange(bounds.min_y, bounds.max_y + self.spacing, self.spacing):
            for x in np.arange(bounds.min_x, bounds.max_x + self.spacing, self.spacing):
                if (x, y) in self.nodes:
                    node = self.get_node(x, y)
                    if node in self.valid_nodes and obstacle.collides_with(node):
                        self.valid_nodes.remove(node)

    def add_obstacles(self, obstacles: list) -> None:
        for obstacle in obstacles:
            self.add_obstacle(obstacle)

    def inflate(self, inflation_radius: float) -> None:
        """Inflates the grid's bounding box and all obstacles within it by the given radius"""

        self.inflation_radius = inflation_radius
        self.bounds.inflate(-inflation_radius)
        
        for obstacle in self.obstacles:
            obstacle.inflate(inflation_radius)
            self.invalidate_neighboring_nodes(obstacle)
    
    def node_valid(self, node: Node) -> bool:
        """Returns true if the given node is valid. 
        Meaning it is within the grid bounds and does not collide with any obstacles"""
        return node in self.valid_nodes and self.bounds.contains_node(node) 
    def a_star(self, start: Node, end: Node) -> tuple[list[float], list[float]]:
        """Finds the shortest path on a grid using A* algorithm"""
        return self.dijkstras(start, end, True)
    
    def get_neighbors(self, node: Node) -> list[Node]:
        """Returns a list of neighboring nodes"""
        neighbors = []
        for y in np.arange(node.y - self.spacing, node.y + 2 * self.spacing, self.spacing):
            for x in np.arange(node.x - self.spacing, node.x + 2 * self.spacing, self.spacing):
                if ((x, y) in self.nodes):
                    neighbor = self.nodes[(x,y)]
                    if (neighbor is not node and self.node_valid(neighbor)):
                        neighbors.append(neighbor)
        return neighbors
    
    def RRT(self, start: Node, end: Node) -> tuple[list[float], list[float]]:
        start.cost = 0
        visited_tree = [start]
        """Finds the shortest path using RRT algorithm"""

        iterations = 0
        while iterations < 10000:
            iterations += 1
            random_node = random.choice(list(self.valid_nodes))

            closest_node = min(visited_tree, key=lambda x: x.distance(random_node))

            # get the angle between the closest node and the random node
            angle = math.atan2(random_node.y - closest_node.y, random_node.x - closest_node.x)
            
            new_x = closest_node.x + self.spacing * math.cos(angle)
            new_y = closest_node.y + self.spacing * math.sin(angle)

            new_node = self.get_node(new_x, new_y)

            if (not self.node_valid(new_node)):
                continue

            if (new_node in visited_tree):
                continue

            new_node.parent_node = closest_node
            new_node.cost = closest_node.cost + closest_node.distance(new_node)
            visited_tree.append(new_node)

            closest_node.visited = True

            # for neighbor in self.get_neighbors(new_node):
            #     if new_node.cost + neighbor.distance(new_node) < neighbor.cost:
            #         neighbor.cost = new_node.cost + neighbor.distance(new_node)
            #         neighbor.parent_node = new_node
            #         visited_tree.append(neighbor)

            if new_node == end or end in self.get_neighbors(new_node):
                end.parent_node = new_node
                end.cost = new_node.cost + new_node.distance(end)
                break
        
        tree_lines = []

        seen = set()

        for node in visited_tree:
            x_list = []
            y_list = []
            while (node.parent_node is not None and node not in seen):
                seen.add(node)
                x_list.append(node.x)
                x_list.append(node.parent_node.x)
                y_list.append(node.y)
                y_list.append(node.parent_node.y)

                node = node.parent_node
            else:
                plt.plot(x_list, y_list, color="green", linewidth=0.5)
                tree_lines.append((x_list, y_list))
        
        # will store x and y coordinates of nodes in path
        x_list = []
        y_list = []

        # start from the end node and work backwars to the start node
        current_node = min(visited_tree, key=lambda x: x.distance(end))
        while current_node != None:
            x_list.append(current_node.x)
            y_list.append(current_node.y)

            current_node = current_node.parent_node
        
        # plot cost of start and end
        # plt.text(start.x, start.y, str(round(start.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")
        # plt.text(end.x, end.y, str(round(end.cost, 2)), color="red", fontsize=8, horizontalalignment="center", verticalalignment = "center")

        return x_list, y_list
    
    def dijkstras(self, start, end, use_heuristic = False) -> tuple[list[float], list[float]]:
        """Finds the shortest path on a grid using Dijkstra's algorithm"""

        reduced_nodes = PriorityQueue()
        reduced_nodes.put((0, start))
        start.cost = 0

        current_node : Node = start

        unvisited = set(self.nodes.values())
        # to reduce search space we only consider nodes that we have "seen" before
        visited = set()

        # loop through unvisited nodes
        while not reduced_nodes.empty():
            current_node = reduced_nodes.get()[1]
            if current_node in visited:
                continue

            visited.add(current_node)
            
            # get the possible neighbors of the current node
            for y in np.arange(current_node.y - self.spacing, current_node.y + 2 * self.spacing, self.spacing):
                for x in np.arange(current_node.x - self.spacing, current_node.x + 2 * self.spacing, self.spacing):
                    # if there is a neighboring node at that coordinate
                    if ((x, y) in self.nodes):
                        neighbor = self.nodes[(x,y)]
                        # if the neighbor is valid and it is not our current node
                        if (neighbor is not current_node and neighbor not in visited and self.node_valid(neighbor)):

                            # update the cost if its less than previous cost
                            cost = current_node.cost + current_node.distance(neighbor)
                            if (cost < neighbor.cost):
                                neighbor.cost = cost
                                neighbor.parent_node = current_node
                                if use_heuristic:
                                    reduced_nodes.put((cost + neighbor.distance(end), neighbor))
                                else:
                                    neighbor.heruistic = neighbor.distance(end)
                                    reduced_nodes.put((cost, neighbor))
            
        
        # will store x and y coordinates of nodes in path
        x_list = []
        y_list = []

        # start from the end node and work backwars to the start node
        current_node = end
        while current_node != None:
            x_list.append(current_node.x)
            y_list.append(current_node.y)

            current_node = current_node.parent_node

        return x_list, y_list


