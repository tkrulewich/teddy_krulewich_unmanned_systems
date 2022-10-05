from itertools import permutations
import math
import matplotlib.pyplot as plt
import multiprocessing

cities = [(2,2), (5,3), (3,4), (6,4)]

for city in cities:
    plt.scatter(city[0], city[1])


distances = {}

for city1 in cities:
    for city2 in cities:
        if city1 != city2:
            distances[(city1, city2)] = ((city1[0] - city2[0])**2 + (city1[1] - city2[1])**2)**0.5


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

print(f"Shortest path is {minPath} with a distance of {minDistance}")

for i in range(len(minPath)-1):
    plt.plot([minPath[i][0], minPath[i+1][0]], [minPath[i][1], minPath[i+1][1]], color='red')

plt.show()