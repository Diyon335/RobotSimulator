"""
This file initialises, allows the modification of parameters and runs the evolutionary algorithm
"""
from ea.encoding import real_number_encoding
from ea.selection import tournament_selection

genotypes = {}

number_of_genotypes = 5

generations = 100


def initialise():
    """
    This function populates the genotype dictionary by using the specified encoding strategy.
    Each genotype has a key (integer), and a fitness which is initially 0

    :return: None
    """

    for i in range(5):
        genotypes[i] = real_number_encoding(5, -2, 2, integers=True), 0

    print(genotypes)


def run_algorithm():
    """
    Initialises the data, and then runs evaluation, selection, reproduction and mutations for the specified amount
    of generations

    :return: None
    """

    initialise()

    for i in range(generations):
        pass
