"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera

from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_parents, one_point_crossover
from ea.reproduction import generational_replacement, generational_rollover
from ea.evaluation import get_xy_phenotype, cost_rosenbrock, cost_rastrigin

genotype_history = []
genotypes = {}

number_of_genotypes = 50

generations = 100

offspring_per_generation = 2

# Variables for the encoding strategy
genotype_length = 12
genotype_min_range = -10
genotype_max_range = 10
whole_numbers = True

# Variables for the evaluation strategy

# Variables for the selection strategy
tournament_k = 5

# Variables for the reproduction strategy

# Variables for the crossover/mutation strategies


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
mutation_strategy = one_point_crossover


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
    for i in range(number_of_genotypes):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)
        x, y = phenotype_computer(genotype)
        generation.append((x, y))

        genotypes[i] = [genotype, 0, (x, y)]

    genotype_history.append(generation)


def run_algorithm():
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :return: None
    """

    initialise()

    for i in range(generations):

        # EVALUATION
        # For each genotype, compute the fitness
        generation = []
        for genotype_id in genotypes:

            # We don't need to get phenotypes for the first generation since it's already done in initialise()
            if i > 0:
                # Replace old phenotype
                genotype = genotypes[genotype_id][0]
                genotypes[genotype_id][2] = phenotype_computer(genotype)

            # Get phenotype
            x, y = genotypes[genotype_id][2]
            generation.append((x, y))

            # Replace current fitness
            genotypes[genotype_id][1] = -cost_function(x, y)

        genotype_history.append(generation)

        print()
        print("Generation "+str(i))
        print(sorted([(genotypes[individual][1]) for individual in genotypes]))

        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # Holds the IDs of the parents that will be copied
        offspring = []

        for j in range(offspring_per_generation):

            best_parent = selection_strategy(genotypes, k=tournament_k)
            offspring.append(best_parent)

        # REPRODUCTION
        reproduction_strategy(genotypes, offspring)
        print(sorted([(genotypes[individual][1]) for individual in genotypes]))
        print(genotypes)

        # MUTATION / CROSS OVER
        key_list = choose_parents(genotypes)
        print(key_list)
        mutation_strategy(genotypes, key_list)
        for id in key_list:
            genotype = genotypes[id][0]
            genotypes[id][2] = phenotype_computer(genotype)
            x, y = genotypes[id][2]
            genotypes[id][1] = -cost_function(x, y)

        print(sorted([(genotypes[individual][1]) for individual in genotypes]))
        print()


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
