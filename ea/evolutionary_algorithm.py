"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_parents, one_point_crossover
from ea.reproduction import generational_replacement, generational_rollover
from ea.evaluation import get_xy_phenotype, cost_rosenbrock, cost_rastrigin

genotype_history = []
genotypes = {}

number_of_genotypes = 100

generations = 100

offspring_per_generation = 5

# Variables for the encoding strategy
genotype_length = 5
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

    for i in range(number_of_genotypes):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)
        x, y = phenotype_computer(genotype)

        genotypes[i] = [genotype, 0, (x, y)]


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
        for genotype_id in genotypes:

            # We don't need to get phenotypes for the first generation since it's already done in initialise()
            if i > 0:
                # Replace old phenotype
                genotype = genotypes[genotype_id][0]
                genotypes[genotype_id][2] = phenotype_computer(genotype)

            # Get phenotype
            x, y = genotypes[genotype_id][2]

            # Replace current fitness
            genotypes[genotype_id][1] = -cost_function(x, y)

        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # Holds the IDs of the parents that will be copied
        offspring = []

        for j in range(offspring_per_generation):

            best_parent = selection_strategy(genotypes, k=tournament_k)
            offspring.append(best_parent)

        # REPRODUCTION
        reproduction_strategy(genotypes, offspring)

        # MUTATION / CROSS OVER
        mutation_strategy(genotypes, offspring)

        genotype_history.append(genotypes)


def plot_generations(i, ax):

    generation = genotype_history[i]
    x, y = [], []
    for genotype_id in generation:
        phenotype_x, phenotype_y = generation[genotype_id][2]
        x.append(phenotype_x)
        y.append(phenotype_y)

    ax.scatter(x, y, c="#000000")
    ax.set_xlim([genotype_min_range, genotype_max_range])
    ax.set_ylim([genotype_min_range, genotype_max_range])
    ax.set_title(f"Generation: {i+1}")


def animate_evolution():
    fig, ax = plt.subplots(nrows=1)

    x_heatmap = np.arange(genotype_min_range, genotype_max_range, 0.01)
    y_heatmap = np.arange(genotype_min_range, genotype_max_range, 0.01)

    z_heatmap = np.zeros((len(x_heatmap), len(y_heatmap)))
    for n in range(len(x_heatmap)):
        for j in range(len(y_heatmap)):
            z_heatmap[n][j] = cost_function(x_heatmap[n], y_heatmap[j])

    im = ax.imshow(z_heatmap, cmap=plt.cm.get_cmap('rainbow'),
                   extent=(genotype_min_range, genotype_max_range, genotype_min_range, genotype_max_range))

    fig.colorbar(im, orientation='vertical')
    print(genotype_history)
    # plot_generations(1, ax)
    anim = animation.FuncAnimation(fig, plot_generations, fargs=(ax,), frames=len(genotype_history), interval=0.05)
    plt.show()

