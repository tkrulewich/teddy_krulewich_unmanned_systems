from homework5.submodules.PathPlanning import *
import matplotlib.pyplot as plt
from time import time


def main():
    grid = Grid(0, 15, 0, 15, 0.5)

    obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 7,
        9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]

    obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6,
        6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]

    for x, y in zip(obstacle_x, obstacle_y):
        grid.add_obstacle(Obstacle(x, y, 0.49))
    
    
    grid.inflate(0.5)

    start = grid.nodes[(1,1)]
    end = grid.nodes[(7, 13)]

    t0 = time()
    x, y = grid.a_star(start, end) 
    t1 = time()

    print("A*")
    print(f"\t Time: {t1 - t0}")
    print(f"\t Path Length: {end.cost}")



    grid.reset()

    t0 = time()
    x, y = grid.dijkstras(start, end) 
    t1 = time()

    print("Dijsktras")
    print(f"\t Time: {t1 - t0}")
    print(f"\t Path Length: {end.cost}")


    grid.reset()

    t0 = time()
    x, y = grid.RRT(start, end) 
    t1 = time()

    print("RRT")
    print(f"\t Time: {t1 - t0}")
    print(f"\t Path Length: {end.cost}")
    



if __name__ == '__main__':
    main()