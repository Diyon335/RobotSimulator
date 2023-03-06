"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
import numpy as np
import matplotlib.pyplot as plt
import random
from celluloid import Camera
from matplotlib import colors

from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_pair, one_point_crossover, uniform_crossover, arithmetic_crossover, mutation
from ea.reproduction import generational_rollover
from ea.evaluation import get_xy_phenotype, cost_rosenbrock, cost_rastrigin

output_directory = "animations/ea_benchmark/diyon/"
phenotype_history = []
population_dictionary = {}

population_size = 50

generations = 100

offsprings_per_generation = 30

# Variables for the encoding strategy
genotype_length = 12
genotype_min_range = -10
genotype_max_range = 10
whole_numbers = False

# Variables for the selection strategy
tournament_k = 6

# Variables for the crossover/mutation strategies
mutation_rate = 3


"""
IMPORTANT: 

Any time you change one of these strategies/functions, be sure to check if the arguments you pass are 
actually those that the strategy/functions require

"""
encoding_strategy = real_number_encoding
phenotype_computer = get_xy_phenotype
cost_function = cost_rastrigin
selection_strategy = tournament_selection
reproduction_strategy = generational_rollover


def initialise():
    """
    This function populates the population dictionary by using the specified encoding strategy.
    Each individual has a key (integer), a genotype array, a fitness and a corresponding phenotype.
    The function also adds the phenotypes of all these initial individuals to the phenotype history array

    The dictionary will be of the form:

    key : value
    int : [[list], int, int int]
    <ID> : [genotype], fitness, tuple(phenotype_x, phenotype_y)

    :return: two lists containing max fitness and average fitness at all generations (for testing purposes)
    """

    generation = []
    for individual in range(population_size):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)
        x, y = phenotype_computer(genotype)
        generation.append((x, y))

        population_dictionary[individual] = [genotype, -cost_function(x, y), phenotype_computer(genotype)]

    phenotype_history.append(generation)


def run_algorithm():
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :return: a max fitness array and an average fitness array, both of length equal to number of generations + 1
            (to include "generation 0" after initialisation but before any reproduction has taken place), containing
            max fitness and average fitness of the population at every generation
    """

    max_fitness_history = []
    avg_fitness_history = []

    initialise()

    features = list(population_dictionary.values())
    features.sort(key=lambda feature: feature[1])
    max_fitness_history.append(features[-1][1])
    avg_fitness_history.append(sum([ind[1] for ind in features]) / len(features))

    for i in range(generations):

        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # and momentarily copy them over in a dedicated offspring dictionary
        offspring_dictionary = {}

        for j in range(offsprings_per_generation):

            best_parent = selection_strategy(population_dictionary, k=tournament_k)

            # offspring is initialised (genotype is set as empty, fitness and phenotype are set to 0)
            offspring_dictionary[j] = [[], 0, (0, 0)]

            # genotype of offspring is copied over from parent
            for gene in population_dictionary[best_parent][0]:
                offspring_dictionary[j][0].append(gene)

        # CROSS OVER
        crossover_events = int(random.uniform(0.1, 0.4)*offsprings_per_generation)

        for _ in range(crossover_events):

            offspring_pair = choose_pair(offspring_dictionary)
            rng = random.randint(0, 99)

            if rng < 33:
                one_point_crossover(offspring_dictionary, offspring_pair)
            elif rng < 66:
                uniform_crossover(offspring_dictionary, offspring_pair)
            else:
                arithmetic_crossover(offspring_dictionary, offspring_pair)

        # MUTATION
        for target_offspring in offspring_dictionary:
            mutation(offspring_dictionary, target_offspring, mutation_rate)

        # REPRODUCTION
        # before offsprings are added into the population, but after they have undergone crossover and mutation events
        # (so that their genotype is "finalised"), their fitness and phenotype is computed
        for offspring in offspring_dictionary:

            genotype = offspring_dictionary[offspring][0]
            offspring_dictionary[offspring][2] = phenotype_computer(genotype)

            x, y = offspring_dictionary[offspring][2]
            offspring_dictionary[offspring][1] = -cost_function(x, y)

        reproduction_strategy(population_dictionary, offspring_dictionary)

        # max fitness and average fitness of the population are extracted and appended to the max fitness and
        # average fitness arrays
        features = list(population_dictionary.values())
        features.sort(key=lambda feature: feature[1])
        max_fitness_history.append(features[-1][1])
        avg_fitness_history.append(sum([ind[1] for ind in features])/len(features))

        # EVALUATION
        # For each individual, add phenotype to phenotype history
        # (at generation 0 this is done by initialise())
        generation = []
        for individual in population_dictionary:
            # Get phenotype
            x, y = population_dictionary[individual][2]
            generation.append((x, y))

        phenotype_history.append(generation)

    return max_fitness_history, avg_fitness_history


def animate_evolution():
    """
    A function to animate the plotting of graphs

    :return: None
    """
    fig, ax = plt.subplots()
    camera = Camera(fig)

    n = 100
    X = np.linspace(genotype_min_range, genotype_max_range, n)
    Y = np.linspace(genotype_min_range, genotype_max_range, n)
    X, Y = np.meshgrid(X, Y)

    Z = cost_function(X, Y)

    for i in range(len(phenotype_history)):

        generation = phenotype_history[i]

        # Holds all phenotypes of all genotypes
        x, y = [], []
        for phenotype in generation:
            # Get the phenotype
            phenotype_x, phenotype_y = phenotype
            x.append(phenotype_x)
            y.append(phenotype_y)

        # Plot all phenotypes
        ax.plot(x, y, 'bo', c="#000000")
        ax.set_xlim([genotype_min_range, genotype_max_range])
        ax.set_ylim([genotype_min_range, genotype_max_range])
        ax.text(0.4, 1.01, f"Generation {i+1}", transform=ax.transAxes)
        camera.snap()

    im = plt.contourf(X, Y, Z, levels=50, cmap="jet") if cost_function == cost_rastrigin else \
         plt.pcolor(X, Y, Z, norm=colors.LogNorm(vmin=Z.min(), vmax=Z.max()), cmap='jet', shading='auto')

    fig.colorbar(im, orientation='vertical', extend='max')
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    # Animates
    animation = camera.animate(interval=200, repeat=False, repeat_delay=500)
    # animation.save(output_directory +
    #                f"{cost_function.__name__}_pop{population_size}_offs{offsprings_per_generation}_gens{generations}_"
    #                f"tk{tournament_k}_mr{mutation_rate}.gif")
    plt.show()


def test_helper(results, value, tests):
    """
    A helper function to make code more concise
    Initialises average max fitness and average average fitness arrays, runs the evolutionary algorithm multiple times
    and fills said arrays across iterations and appends said arrays into the dictionary of results

    :param results: The dictionary of results
    :param value: The value for the parameter being investigated
    :param tests: The number of times the evolutionary algorithm should be run
    :return: None
    """

    avg_max_fitnesses = [0 for _ in range(generations+1)]
    avg_avg_fitnesses = [0 for _ in range(generations+1)]
    results[value] = []

    for itter in range(tests):
        max_fitness_history, avg_fitness_history = run_algorithm()
        for i in range(len(max_fitness_history)):
            avg_max_fitnesses[i] += max_fitness_history[i]
            avg_avg_fitnesses[i] += avg_fitness_history[i]

    # the dictionary of results is modified in place
    results[value].append(avg_max_fitnesses)
    results[value].append(avg_avg_fitnesses)


def testing_routine(parameter_set, parameter, tests=100, short=False):
    """
    The function that handles running multiple experiments with different values for a given parameter
    to be investigated

    :param parameter_set: A list of values for the parameter to be investigated
    :param parameter: The parameter of the evolutionary algorithm under investigation
    :param tests: The number of times the evolutionary algorithm should be run to test a single value for the parameter
    :param short: A boolean to indicate whether the user wants to run a simple and quick testing routine

    :return: A dictionary of results, the keys are values for a parameter being investigated in a testing routine
            and values are lists of two arrays, the first containing average max fitness at every generation
            of the genetic algorithm and the second containing average average fitness at every generation
    """

    if short:
        global generations
        generations = 10
        tests = 10

    results = {}

    if parameter == "tournament_k":

        for value in parameter_set:

            global tournament_k
            tournament_k = value
            test_helper(results, value, tests)
            print("Finished with value " + str(value))

    elif parameter == "offsprings_per_generation":

        for value in parameter_set:

            offsprings_per_generation = value
            test_helper(results, value, tests)
            print("Finished with value " + str(value))

    elif parameter == "mutation_rate":

        for value in parameter_set:

            mutation_rate = value
            test_helper(results, value, tests)
            print("Finished with value "+str(value))

    return results
