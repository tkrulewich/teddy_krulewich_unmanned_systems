from itertools import permutations
import math
import matplotlib.pyplot as plt
import multiprocessing

cities = [(0, 0), (2,2), (5,3), (3,4), (6,4)]

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

plt.annotate("Start", minPath[0])

for i in range(1, len(minPath)):
    current_city = minPath[i]
    previous_city = minPath[i-1]

    plt.annotate(f"Goal {cities.index(current_city)}", current_city)
    plt.plot([current_city[0], previous_city[0]], [current_city[1], previous_city[1]], color='red')

plt.show()