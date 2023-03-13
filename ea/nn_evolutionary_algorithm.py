"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
import numpy as np
import matplotlib.pyplot as plt
import random
from celluloid import Camera
from matplotlib import colors
from scipy.stats import sem

from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_pair, one_point_crossover, uniform_crossover, arithmetic_crossover, mutation
from ea.reproduction import generational_rollover
from ea.evaluation import get_xy_phenotype, cost_rosenbrock, cost_rastrigin
from ea.robot_evaluation import evaluate_genotype

import time

output_directory = "animations/ea_benchmark/diyon/"
plot_directory = "plots/"
phenotype_history = []
population_dictionary = {}

population_size = 30

generations = 100

offsprings_per_generation = 25

# Variables for the encoding strategy
genotype_length = 210
genotype_min_range = -10
genotype_max_range = 10
whole_numbers = False

# Variables for the selection strategy
tournament_k = 6

# Variables for the crossover/mutation strategies
mutation_rate = 5

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


def initialise(room):
    """
    This function populates the population dictionary by using the specified encoding strategy.
    Each individual has a key (integer), a genotype array and a fitness.

    The dictionary will be of the form:

    key : value
    int : [[list], int]
    <ID> : [genotype], fitness

    :return: none
    """

    # generation = []
    for individual in range(population_size):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)

        population_dictionary[individual] = [genotype, evaluate_genotype(genotype, individual, room)]

def initialise_rooms(rooms):
    """
    This function populates the population dictionary by using the specified encoding strategy.
    Each individual has a key (integer), a genotype array and a fitness.

    The dictionary will be of the form:

    key : value
    int : [[list], int]
    <ID> : [genotype], fitness

    :return: none
    """

    # generation = []
    for individual in range(population_size):

        genotype = encoding_strategy(genotype_length, genotype_min_range, genotype_max_range, integers=whole_numbers)

        fitness = 0
        for room in rooms:
            print("Room: " + str(room[2]))
            fitness += evaluate_genotype(genotype, individual, room)

        population_dictionary[individual] = [genotype, fitness / len(rooms)]

def run_algorithm(room, file):
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :param room: Room list
    :param file: An open file to write to
    :return: a max fitness array and an average fitness array, both of length equal to number of generations + 1
            (to include "generation 0" after initialisation but before any reproduction has taken place), containing
            max fitness and average fitness of the population at every generation
    """

    max_fitness_history = []
    avg_fitness_history = []

    start = time.time()
    print(f"Running for {generations} generations with {population_size} genotypes\n"
          f"Initialising all genotypes")

    file.write(f"This file contains a run for {generations} generations with {population_size} genotypes\n"
               f"offspring per generation: {offsprings_per_generation}\n"
               f"k: {tournament_k}\n"
               f"mutation rate: {mutation_rate}\n\n")

    initialise(room)

    print(f"Done initialising in {time.time() - start} seconds")

    file.write("Initialised genotypes:\n")
    write_dictionary(population_dictionary, file)
    file.write("############################\n")

    start = time.time()

    # Get the max and the avg fitness at the start
    features = [individual[1] for individual in population_dictionary.values()]
    max_fitness_history.append(max(features))
    avg_fitness_history.append(sum(features) / len(features))

    for i in range(generations):

        print(f"Evaluating generation: {i+1}")
        file.write(f"GENERATION: {i+1}\n")

        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # and momentarily copy them over in a dedicated offspring dictionary
        offspring_dictionary = {}

        for j in range(offsprings_per_generation):

            best_parent = selection_strategy(population_dictionary, k=tournament_k)

            # offspring is initialised (genotype is set as empty, fitness is set to 0)
            offspring_dictionary[j] = [[], 0]

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
                arithmetic_crossover(offspring_dictionary, offspring_pair, integers=False)

        # MUTATION
        for target_offspring in offspring_dictionary:
            mutation(offspring_dictionary, target_offspring, mutation_rate, integers=False)

        # REPRODUCTION
        # before offsprings are added into the population, but after they have undergone crossover and mutation events
        # (so that their genotype is "finalised"), their fitness and phenotype is computed
        for offspring in offspring_dictionary:

            genotype = offspring_dictionary[offspring][0]

            offspring_dictionary[offspring][1] = evaluate_genotype(genotype, offspring, room)

        reproduction_strategy(population_dictionary, offspring_dictionary)

        # max fitness and average fitness of the population are extracted and appended to the max fitness and
        # average fitness arrays
        features = [individual[1] for individual in population_dictionary.values()]

        max_fitness = max(features)
        avg_fitness = sum(features) / len(features)
        max_fitness_history.append(max_fitness)
        avg_fitness_history.append(avg_fitness)

        end = time.time()
        print(f"Finished generation {i+1} in {end - start} seconds.\n "
              f"Max fitness: {max_fitness}\n "
              f"Avg. fitness: {avg_fitness}")
        
        start = time.time()

        write_dictionary(population_dictionary, file)
        file.write("############################\n")
    
    best_genotype = max(population_dictionary.values(), key=lambda x: x[1])
    print(f"The best genotype is:\n"
          f"{best_genotype}")

    file.write(f"The best genotype is: {best_genotype}")

    return max_fitness_history, avg_fitness_history


def write_dictionary(dictionary, file):

    for genotype_id in dictionary:

        line = f"ID: {str(genotype_id)}\n" \
               f"Genotype: {str(dictionary[genotype_id][0])}\n" \
               f"Fitness: {str(dictionary[genotype_id][1])}\n\n"

        file.write(line)
        file.write("------------------------------------\n")


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


def test_helper(results, value, tests, room):
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
        max_fitness_history, avg_fitness_history = run_algorithm(room)
        for i in range(len(max_fitness_history)):
            avg_max_fitnesses[i] += max_fitness_history[i]
            avg_avg_fitnesses[i] += avg_fitness_history[i]

    # the dictionary of results is modified in place
    results[value].append(avg_max_fitnesses)
    results[value].append(avg_avg_fitnesses)


def testing_routine(parameter_set, parameter, room, tests=100, short=False):
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
            test_helper(results, value, tests, room)
            print("Finished with value " + str(value))

    elif parameter == "offsprings_per_generation":

        for value in parameter_set:

            global offsprings_per_generation
            offsprings_per_generation = value
            test_helper(results, value, tests, room)
            print("Finished with value " + str(value))

    elif parameter == "mutation_rate":

        for value in parameter_set:

            global mutation_rate
            mutation_rate = value
            test_helper(results, value, tests, room)
            print("Finished with value "+str(value))

    return results


def run_algorithm_with_parameters(rooms, gens, k, num_offspring, mr, file):
    """
    This function allows the value of parameters to be changed by passing in values

    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :param file: An open file object
    :param mr: Integer for mutation rate
    :param num_offspring: Integer for number of offspring per generation
    :param k: Integer for tournament k
    :param gens: Integer for number of generations
    :param room: Room list

    :return: a max fitness array and an average fitness array, both of length equal to number of generations + 1
            (to include "generation 0" after initialisation but before any reproduction has taken place), containing
            max fitness and average fitness of the population at every generation
    """

    max_fitness_history = []
    avg_fitness_history = []

    start = time.time()
    print(f"Running for {gens} generations with {population_size} genotypes\n"
          f"Initialising all genotypes")

    file.write(f"This file contains a run for {gens} generations with {population_size} genotypes\n"
               f"offspring per generation: {num_offspring}\n"
               f"k: {k}\n"
               f"mutation rate: {mr}\n\n")

    initialise_rooms(rooms)

    print(f"Done initialising in {time.time() - start} seconds")

    file.write("Initialised genotypes:\n")
    write_dictionary(population_dictionary, file)
    file.write("############################\n")

    start = time.time()

    # Get the max and the avg fitness at the start
    features = [individual[1] for individual in population_dictionary.values()]
    max_fitness_history.append(max(features))
    avg_fitness_history.append(sum(features) / len(features))

    for i in range(gens):

        print(f"Evaluating generation: {i + 1}")
        file.write(f"GENERATION: {i + 1}\n")

        # SELECTION
        # For the number of desired offspring per generation, choose the best parent
        # and momentarily copy them over in a dedicated offspring dictionary
        offspring_dictionary = {}

        for j in range(num_offspring):

            best_parent = selection_strategy(population_dictionary, k=k)

            # offspring is initialised (genotype is set as empty, fitness is set to 0)
            offspring_dictionary[j] = [[], 0]

            # genotype of offspring is copied over from parent
            for gene in population_dictionary[best_parent][0]:
                offspring_dictionary[j][0].append(gene)

        # CROSS OVER
        crossover_events = int(random.uniform(0.1, 0.4)*num_offspring)

        for _ in range(crossover_events):

            offspring_pair = choose_pair(offspring_dictionary)
            rng = random.randint(0, 99)

            if rng < 33:
                one_point_crossover(offspring_dictionary, offspring_pair)
            elif rng < 66:
                uniform_crossover(offspring_dictionary, offspring_pair)
            else:
                arithmetic_crossover(offspring_dictionary, offspring_pair, integers=False)

        # MUTATION
        for target_offspring in offspring_dictionary:
            mutation(offspring_dictionary, target_offspring, mr, integers=False)

        # REPRODUCTION
        # before offsprings are added into the population, but after they have undergone crossover and mutation events
        # (so that their genotype is "finalised"), their fitness and phenotype is computed
        for offspring in offspring_dictionary:

            genotype = offspring_dictionary[offspring][0]
            fitness = 0
            for room in rooms:
                fitness += evaluate_genotype(genotype, offspring, room)
            offspring_dictionary[offspring][1] = fitness / len(rooms)

        reproduction_strategy(population_dictionary, offspring_dictionary)

        # max fitness and average fitness of the population are extracted and appended to the max fitness and
        # average fitness arrays
        features = [individual[1] for individual in population_dictionary.values()]

        max_fitness = max(features)
        avg_fitness = sum(features) / len(features)
        max_fitness_history.append(max_fitness)
        avg_fitness_history.append(avg_fitness)

        end = time.time()
        print(f"Finished generation {i+1} in {end - start} seconds.\n "
              f"Max fitness: {max_fitness}\n "
              f"Avg. fitness: {avg_fitness}")

        write_dictionary(population_dictionary, file)
        file.write("############################\n")

        start = time.time()

    best_genotype = max(population_dictionary.values(), key=lambda x: x[1])
    print(f"The best genotype is:\n"
          f"{best_genotype}")

    file.write(f"The best genotype is: {best_genotype}")
    file.write(f"\n")
    file.write(f"{max_fitness_history}\n")
    file.write(f"{avg_fitness_history}\n")

    return max_fitness_history, avg_fitness_history


def test_algorithm_with_parameters(room, file, plot_name, gens=generations, k=tournament_k, num_offspring=offsprings_per_generation, mr=mutation_rate, tests=100):
    """
    Plots the average max fitness and average fitness from all run tests over a number of generations.

    Plots contain error bars

    :param room: Room list
    :param gens: Integer for number of generations
    :param k: Integer for tournament k
    :param num_offspring: Integer for number of offspring per generation
    :param mr: Integer for mutation rate
    :param tests: Integer for number of tests
    :return: None - saves figure
    """
    # Holds lists of max and avg fitness per run test
    max_fitness = []
    avg_fitness = []

    print(f"Running the algorithm for {tests} tests, each containing {gens} generations.\n"
          f"Parameters:\n"
          f"k = {tournament_k}\n"
          f"offspring per gen = {num_offspring}\n"
          f"mutation rate = {mr}")

    for i in range(tests):

        print(f"TEST {i+1}:")
        file.write(f"TEST {i+1}\n")

        max_fitness_history, avg_fitness_history = run_algorithm_with_parameters(room, gens, k, num_offspring, mr, file)

        max_fitness.append(max_fitness_history)
        avg_fitness.append(avg_fitness_history)

        file.write("##############################################\n")

    # Holds the average max fitness and average average fitness over all tests, per generation
    avg_max_fitness = []
    avg_avg_fitness = []

    # Holds the errors
    max_fitness_error = []
    avg_fitness_error = []

    # For each generation, go over all run tests and compute the average + errors
    for j in range(gens):

        total_max_fitness = []
        total_avg_fitness = []

        for k in range(tests):

            total_max_fitness.append(max_fitness[k][j])
            total_avg_fitness.append(avg_fitness[k][j])

        avg_max_fitness.append(sum(total_max_fitness) / tests)
        avg_avg_fitness.append(sum(total_avg_fitness) / tests)

        max_fitness_error.append(sem(total_max_fitness))
        avg_fitness_error.append(sem(total_avg_fitness))

    generation_list = [gen for gen in range(gens)]

    # Plot
    plt.errorbar(generation_list, avg_max_fitness, yerr=max_fitness_error, c='b', ls='-', fmt='.', label="Max Fitness")
    plt.errorbar(generation_list, avg_avg_fitness, yerr=avg_fitness_error, c='r', ls='-', fmt='.', label="Average Fitness")

    plt.xlabel("Generations")
    plt.ylabel("Fitness")

    plt.xlim(0, gens)
    plt.legend(loc='lower right')

    # Save
    plt.savefig(plot_directory+plot_name)

