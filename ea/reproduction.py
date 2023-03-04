"""
This file contains reproduction strategies
"""
import operator


def generational_replacement(genotype_dictionary, offspring):
    """
    Modifies the dictionary provided

    Replaces all genotypes with the offspring

    :param offspring: List of IDs of the offspring/best parents
    :param genotype_dictionary: The dictionary containing all genotypes
    :return: None
    """

    # The new population size must be equal to the old population size
    if len(offspring) != len(genotype_dictionary):
        raise Exception("The number of new offspring does not match the number of old genotypes")

    # Build a new list of offspring
    offspring_list = []

    for genotype_id in offspring:
        offspring_list.append(genotype_dictionary[genotype_id])

    genotype_dictionary.clear()

    # Copy elements from offspring dictionary to genotype dictionary
    for i, child in enumerate(offspring_list):
        genotype_dictionary[i] = child


def generational_rollover(genotype_dictionary, offspring):
    """
    Modifies the dictionary provided

    Replaces the n worst genotypes in the generation, where n is the number of offspring per generation

    :param offspring: List of IDs of the offspring/best parents
    :param genotype_dictionary: The dictionary containing all genotypes
    :return: None
    """

    n = len(offspring)

    # Lowest fitness --> Highest fitness
    genotype_list = sorted(genotype_dictionary.items(), key=lambda genotype: genotype[1][1], reverse=False)

    # Get the IDs (index 0 in the list) of the worst individuals starting from the worst to the nth worst
    worst_individuals = [genotype_list[i][0] for i in range(0, n)]

    # Replace each worst ID with the offspring
    for good_parent, bad_individual in zip(offspring, worst_individuals):
        genotype_dictionary[bad_individual] = genotype_dictionary[good_parent]

