"""
This file contains selection strategies for the evolutionary algorithm
"""
import random
from ea.reproduction import reproduction_tournament_selection

def tournament_selection(genotype_dictionary, offspings_amount, k=5):

    """
    Tournament selection strategy takes a sample, size k, from all the genotypes and selects the best genotype

    :param genotype_dictionary: The dictionary containing all genotypes
    :param k: An integer indicating the sample size
    :param offsrpings_amount: An integer indicating how many offspring have to be generated
    :return: Returns an integer indicating the best genotype. The integer is the dictionary key of the best genotype
    """

    for i in range(offspings_amount):
        list = random.sample(range(0, len(genotype_dictionary)), 5)
        max = -1
        index = -1

        print(list)

        for i in list:
            if genotype_dictionary[i][1] > max:
                max = genotype_dictionary[i][1]
                index = i

        reproduction_tournament_selection(genotype_dictionary, index)

    return index
