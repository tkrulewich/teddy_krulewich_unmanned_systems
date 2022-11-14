from exam.submodules.PathPlanning import Grid, Obstacle
import math
from itertools import permutations
import matplotlib.pyplot as plt
import time

class Gene:
    def __init__(self, path, grid : Grid):
        self.path = path
        self.distance = None

        self.path =  grid.a_star(grid.nodes[path[0]], grid.nodes[path[1]])
    
    def __lt__(self, other):
        return self.distance < other.distance
    


def main():
    pass

if __name__ == "__main__":
    main()