from exam.submodules.PathPlanning import Grid, Obstacle
import math
import matplotlib.pyplot as plt
import time
import numpy as np

from copy import deepcopy

class Chromosome:
    def __init__(self, GA, path = None):
        self.GA = GA

        if path is None:
            city_indices = np.random.permutation(range(1, len(self.GA.cities)))

            self.path = [self.GA.cities[0]]
            for i in range(len(city_indices)):
                city = self.GA.cities[city_indices[i]]
                self.path.append(city)

            self.path = tuple(self.path)
        else:
            self.path = path
        
        self.distance = 0

        for i in range(len(self.path)-1):
            city = self.path[i]
            next_city = self.path[i+1]
            self.distance += self.GA.distances[(city, next_city)]



        self.fitness = -self.distance

        self.normalized_fitness = None

        self.mutations = [self.__swap_genes__, self.__shuffle_section__, self.__shift_left__, self.__shift_right__, self.__move_gene__]
        
    def __lt__(self, other):
        return self.fitness < other.fitness
    
    def mutate(self):
        random_mutate = np.random.randint(0, len(self.mutations))

        return self.mutations[random_mutate]()

    
    def __swap_genes__(self):
        new_path = list(self.path)

        index1 = np.random.randint(1, len(new_path))
        index2 = np.random.randint(1, len(new_path))

        new_path[index1], new_path[index2] = new_path[index2], new_path[index1]

        return Chromosome(self.GA, tuple(new_path))

    
    def __shuffle_section__(self):
        new_path = list(self.path)

        start_index = np.random.randint(1, len(new_path))
        end_index = np.random.randint(start_index, len(new_path))

        new_slice = np.random.permutation(new_path[start_index:end_index])

        for i in range(start_index, end_index):
            new_path[i] = (new_slice[i-start_index, 0], new_slice[i-start_index, 1])
        
        return Chromosome(self.GA, tuple(new_path))

    def __shift_left__(self):
        new_path = list(self.path)
        new_path = [new_path[0], new_path[-1]] + new_path[1:-1]

        return Chromosome(self.GA, tuple(new_path))
    
    def __shift_right__(self):
        new_path = list(self.path)
        new_path = [new_path[0]] + new_path[2:] + [new_path[1]]

        return Chromosome(self.GA, tuple(new_path))
    
    def __move_gene__(self):
        new_path = list(self.path)

        index1 = np.random.randint(1, len(new_path))
        gene = new_path.pop(index1)

        index2 = np.random.randint(1, len(new_path))

        new_path.insert(index2, gene)

        return Chromosome(self.GA, tuple(new_path))
    
    def crossover(self, gene):
        child1_path = list(self.path)
        child2_path = list(gene.path)

        split_index = np.random.randint(1, len(child1_path))

        child1_path[split_index:] = gene.path[split_index:]
        child2_path[split_index:] = self.path[split_index:]

        city_set_1 = set()
        city_set_2 = set()

        all_cities_1 = set(self.GA.cities)
        all_cities_2 = set(self.GA.cities)

        empty_cities_1 = set()
        empty_cities_2 = set()

        for i in range(len(child1_path)):
            if child1_path[i] in city_set_1:
                child1_path[i] = None
                empty_cities_1.add(i)
            else:
                city_set_1.add(child1_path[i])
            if child2_path[i] in city_set_2:
                child2_path[i] = None
                empty_cities_2.add(i)
            else:
                city_set_2.add(child2_path[i])
        
        for i in empty_cities_1:
            child1_path[i] = all_cities_1.difference(city_set_1).pop()
            city_set_1.add(child1_path[i])
            all_cities_1.remove(child1_path[i])
        
        for i in empty_cities_2:
            child2_path[i] = all_cities_2.difference(city_set_2).pop()
            city_set_2.add(child2_path[i])
            all_cities_2.remove(child2_path[i])

        return Chromosome(self.GA, tuple(child1_path)), Chromosome(self.GA, tuple(child2_path))






class GeneticAlgorithm:
    def __init__(self, population_size, mutation_rate, crossover_rate, cities, grid : Grid):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.total_fitness = 0

        self.cities = cities
        self.grid = grid

        self.cities = cities

        self.population = []

        self.start_time = time.time()
        
        self.distances = {}
        self.paths = {}

        # generate initial population
        for city1 in cities:
            for city2 in cities:
                if city1 != city2:
                    if (city2, city1) in self.distances:
                        self.distances[(city1, city2)] = self.distances[(city2, city1)]
                        self.paths[(city1, city2)] = self.paths[(city2, city1)]
                        continue
                    x, y = grid.a_star(grid.nodes[city1], grid.nodes[city2])

                    self.paths[(city1, city2)] = (x, y)

                    distance = grid.nodes[city2].cost

                    grid.reset_nodes()

                    self.distances[(city1, city2)] = distance
        
        self.init_population()

    def init_population(self):
        for i in range(self.population_size):
            self.population.append(Chromosome(self))

    
    def run(self, iterations):

        best_fitness = -math.inf
        for i in range(iterations):
            self.population.sort()
            
            worst_fitness = self.population[0].fitness

            self.total_fitness = 0
            for gene in self.population:
                gene.normalized_fitness = gene.fitness - worst_fitness
                self.total_fitness += gene.normalized_fitness
            

            self.next_generation = self.population[-1:-5:-1]
            random_lucky_few = np.random.choice(self.population, 5, replace=False)

            for gene in random_lucky_few:
                self.next_generation.append(gene)

            for j in range(25):
                self.next_generation.append(Chromosome(self))
            

            while len(self.next_generation) < self.population_size:
                parent1, parent2 = self.select_parents()

                if np.random.random() < self.crossover_rate:
                    child1, child2 = parent1.crossover(parent2)
                else:
                    child1, child2 = parent1, parent2

                if np.random.random() < self.mutation_rate:
                    child1 = child1.mutate()
                if np.random.random() < self.mutation_rate:
                    child2 = child2.mutate()

                self.next_generation.append(child1)
                self.next_generation.append(child2)
            
            if self.population[-1].fitness > best_fitness:
                best_fitness = self.population[-1].fitness

                self.best_solution_time = time.time() - self.start_time
                print("Iteration: " + str(i) + " Best Distance: " + str(self.population[-1].distance))
                print("\t Best Path: " + str(self.population[-1].path))
                print(f"\t Time: {self.best_solution_time:.2f} seconds")
                

            self.population = self.next_generation
        
        final_time = time.time() - self.start_time
        print(f"{iterations} iterations completed in {final_time:.2f} seconds")
        
    
    def select_parents(self, k=2):
        parents = set()

        while len(parents) < 2:
            candidates = np.random.choice(self.population, k, replace=False, p=[gene.normalized_fitness / self.total_fitness for gene in self.population])

            candidates.sort()

            parents.add(candidates[-1])
        

        return list(parents)

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
    
    GA = GeneticAlgorithm(500, 0.5, 0.7, cities, grid)

    GA.run(2000)

    
    # print("Best fitness: ", best_gene.fitness)
    # print(f" \t path: {best_gene.path}")
    # print(f" \t distance: {best_gene.distance}")

    # print("------------------------")
    # print("Worst fitness: ", worst_gene.fitness)
    # print(f" \t path: {worst_gene.path}")
    # print(f" \t distance: {worst_gene.distance}")
        

if __name__ == "__main__":
    main()