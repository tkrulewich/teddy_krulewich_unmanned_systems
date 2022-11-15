from exam.submodules.PathPlanning import Grid, Obstacle
import math
from itertools import permutations
import matplotlib.pyplot as plt
import time

def main():
    ox = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 
        8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2,  2,  2,  2,  5, 5,  5,  5,  5, 
        6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 12]
    oy = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2,  2,  2,  2,  3, 4, 5, 6, 
        7, 8, 9, 7, 7, 7, 7, 7, 6, 6,  6,  6,  6,  8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
        12, 12, 12, 12, 12, 12, 8,  9,  10, 11, 12]

    grid = Grid(0, 15, 0, 15, 0.5)

    for i in range(len(ox)):
        grid.add_obstacle(Obstacle(ox[i], oy[i], 0.5))
    
    grid.inflate(0.5)

    cities = [(0,0),  (1,1), (9,7), (1,9), (4,4), (9,4), (6,14), (3,11), (14,1), (1,14), (14,14), (7,10) ]
    paths = {}

    distances = {}

    t0 = time.time()

    for city1 in cities:
        for city2 in cities:
            if city1 != city2:
                if (city2, city1) in distances:
                    distances[(city1, city2)] = distances[(city2, city1)]
                    paths[(city1, city2)] = paths[(city2, city1)]
                    continue
                x, y = grid.a_star(grid.nodes[city1], grid.nodes[city2])

                paths[(city1, city2)] = (x, y)

                distance = grid.nodes[city2].cost

                grid.reset_nodes()

                distances[(city1, city2)] = distance
    
    minDistance = math.inf
    minPath = None

    for path in permutations(cities[1:]):
        path = (cities[0],) + path
        
        total_distance = 0
        for i in range(len(path)-1):
            total_distance += distances[(path[i], path[i+1])]
        
        if total_distance < minDistance:
            minDistance = total_distance
            minPath = path

    t1 = time.time()
    
    print(f"Shortest path is {minPath} with a distance of {minDistance}")
    print(f"Time taken: {t1-t0} seconds")

    grid.draw()

    # plot the minimum path

    for i in range(1, len(minPath)):
        current_city = minPath[i]
        previous_city = minPath[i-1]

        plt.annotate(f"Goal {cities.index(current_city)}", current_city)

        plt.plot(paths[(previous_city, current_city)][0], paths[(previous_city, current_city)][1], 'r')

    

    plt.show()





if __name__ == '__main__':
    main()