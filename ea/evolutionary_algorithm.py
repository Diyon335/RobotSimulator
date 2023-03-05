"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
import numpy as np
import matplotlib.pyplot as plt
import copy
import random
from celluloid import Camera
from collections import Counter

from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_pair, one_point_crossover, uniform_crossover, arithmetic_crossover, mutation
from ea.reproduction import generational_replacement, generational_rollover
from ea.evaluation import get_xy_phenotype, cost_rosenbrock, cost_rastrigin

genotype_history = []
population_dictionary = {}

population_size = 50

generations = 100

offsprings_per_generation = 30

# Variables for the encoding strategy
genotype_length = 12
genotype_min_range = -10
genotype_max_range = 10
whole_numbers = False

# Variables for the evaluation strategy

# Variables for the selection strategy
tournament_k = 6

# Variables for the reproduction strategy

# Variables for the crossover/mutation strategies
mutation_rate = 3


"""
IMPORTANT: 

Any time you change one of these strategies/functions, be sure to check if the arguments you pass are 
actually those that the strategy/functions require

"""
encoding_strategy = real_number_encoding
phenotype_computer = get_xy_phenotype
cost_function = cost_rosenbrock
selection_strategy = tournament_selection
reproduction_strategy = generational_rollover


def initialise():
    """
    This function populates the genotype dictionary by using the specified encoding strategy.
    Each genotype has a key (integer), a fitness which is initially 0, and its corresponding phenotype

    The dictionary will be of the form:

    key : value
    int : [[list], int, int int]
    <ID> : [genotype], fitness, tuple(phenotype_x, phenotype_y)

    :return: None
    """

    generation = []
    for individual in range(population_size):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)
        x, y = phenotype_computer(genotype)
        generation.append((x, y))

        population_dictionary[individual] = [genotype, 0, (x, y)]

    genotype_history.append(generation)


def run_algorithm(placeholder):
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations
    :param placeholder: Some value passed into the algorithm for the purpose of testing
    :return: Some information about the current run of the algorithm
    """

    ## Change which variable to set to the passed parameter to explore different options
    mutation_rate = placeholder
    initialise()

    for i in range(generations):

        # EVALUATION
        # For each genotype, compute the fitness
        generation = []
        for individual in population_dictionary:

            # We don't need to get phenotypes for the first generation since it's already done in initialise()
            if i > 0:
                # Replace old phenotype
                genotype = population_dictionary[individual][0]
                population_dictionary[individual][2] = phenotype_computer(genotype)

            # Get phenotype
            x, y = population_dictionary[individual][2]
            generation.append((x, y))

            # Replace current fitness
            population_dictionary[individual][1] = -cost_function(x, y)

        genotype_history.append(generation)

        ## Stoping condition:
        ##      return early when >50% of individuals have the same genotype
        pop_list = list(population_dictionary.values())
        genotype_list = [item[0] for item in pop_list]
        
        counter = Counter(map(tuple, genotype_list))
        print(counter.most_common(1))
        if counter.most_common(1)[0][1] > population_size * 0.6:
            return max(pop_list, key=lambda x: x[1])[1], i
        


        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # and momentarily copy them over in a dedicated offspring dictionary
        offspring_dictionary = {}

        for j in range(offsprings_per_generation):

            best_parent = selection_strategy(population_dictionary, k=tournament_k)
            # offspring_dictionary[j] = copy.copy(population_dictionary[best_parent])
            offspring_dictionary[j] = [[], 0, (0, 0)]
            for gene in population_dictionary[best_parent][0]:
                offspring_dictionary[j][0].append(gene)

        # MUTATION / CROSS OVER
        crossover_events = int(random.uniform(0.1, 0.4)*offsprings_per_generation)
        # crossover_events = 1
        for _ in range(crossover_events):
            offspring_pair = choose_pair(offspring_dictionary)
            rng = random.randint(0, 99)
            if rng < 33:
                one_point_crossover(offspring_dictionary, offspring_pair)
            elif rng < 66:
                uniform_crossover(offspring_dictionary, offspring_pair)
            else:
                arithmetic_crossover(offspring_dictionary, offspring_pair)

        for target_offspring in offspring_dictionary:
            mutation(offspring_dictionary, target_offspring, mutation_rate)

        # REPRODUCTION
        for offspring in offspring_dictionary:
            genotype = offspring_dictionary[offspring][0]
            offspring_dictionary[offspring][2] = phenotype_computer(genotype)
            x, y = offspring_dictionary[offspring][2]
            offspring_dictionary[offspring][1] = -cost_function(x, y)
        reproduction_strategy(population_dictionary, offspring_dictionary)

        # # print(population_dictionary)
        # features = list(population_dictionary.values())
        # features.sort(key=lambda feature: feature[1])
        # print([(ind[1], ind[2]) for ind in features])
    
    pop_list = list(population_dictionary.values())
    
    return max(pop_list, key=lambda x: x[1])[1], generations


def animate_evolution():
    """
    A function to animate the plotting of graphs

    :return: None
    """
    fig, ax = plt.subplots()
    camera = Camera(fig)

    # Create x, y values from min --> max in steps of 0.01
    x_heatmap = np.arange(genotype_min_range, genotype_max_range, 0.01)
    y_heatmap = np.arange(genotype_min_range, genotype_max_range, 0.01)

    # Get the cost of every x, y combination
    z_heatmap = np.zeros((len(x_heatmap), len(y_heatmap)))
    for n in range(len(x_heatmap)):
        for j in range(len(y_heatmap)):
            z_heatmap[n][j] = cost_function(x_heatmap[n], y_heatmap[j])

    for i in range(len(genotype_history)):
        generation = genotype_history[i]

        # Holds all phenotypes of all genotypes
        x, y = [], []
        for phenotype in generation:
            # Get the phenotype
            phenotype_x, phenotype_y = phenotype
            x.append(phenotype_x)
            y.append(phenotype_y)

        # Plot all phenotypes
        ax.scatter(x, y, c="#000000")
        ax.set_xlim([genotype_min_range, genotype_max_range])
        ax.set_ylim([genotype_min_range, genotype_max_range])
        ax.text(0.4, 1.01, f"Generation {i+1}", transform=ax.transAxes)
        camera.snap()

    im = ax.imshow(z_heatmap, cmap=plt.cm.get_cmap('rainbow'),
                   extent=(genotype_min_range, genotype_max_range, genotype_min_range, genotype_max_range))

    fig.colorbar(im, orientation='vertical')

    # Animates
    animation = camera.animate(interval=200, repeat=True, repeat_delay=500)
    plt.show()
