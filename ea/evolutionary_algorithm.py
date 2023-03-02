"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
from ea.encoding import real_number_encoding
from ea.selection import tournament_selection
from ea.crossover_mutation import choose_parents, one_point_crossover
from ea.selection import tournament_selection
from ea.reproduction import reproduction_tournament_selection

genotypes = {}

number_of_genotypes = 5

generations = 100

# Variables for the encoding strategy
genotype_length = 5
genotype_min_range = -2
genotype_max_range = 2
whole_numbers = True

# Variables for the evaluation strategy

# Variables for the selection strategy
tournament_k = 5

# Variables for the reproduction strategy

# Variables for the crossover/mutation strategies


def initialise():
    """
    This function populates the genotype dictionary by using the specified encoding strategy.
    Each genotype has a key (integer), and a fitness which is initially 1

    :return: None
    """

    for i in range(5):
        genotypes[i] = real_number_encoding(genotype_length, genotype_min_range, genotype_max_range, whole_numbers), 0

    print(genotypes)


def run_algorithm():
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :return: None
    """

    initialise()

    print(genotypes)

    tournament_selection(genotypes, 3)

    print(genotypes)

    for i in range(generations):
        pass
