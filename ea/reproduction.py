"""
This file contains reproduction strategies
"""
import operator
import copy


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


def generational_rollover(population_dictionary, offspring_dictionary):
    """
    Modifies the dictionary provided
    Replaces the least fit individuals in the population with the offsprings
    :param population_dictionary: The dictionary containing all individuals in the population
    :param offspring_dictionary: Dictionary of the offspring/best parents
    :return: None
    """

    n = len(offspring_dictionary)

    # Lowest fitness --> Highest fitness
    population_list = sorted(population_dictionary.items(), key=lambda individual: individual[1][1])

    # Get the IDs (index 0 in the list) of the worst individuals starting from the worst to the nth worst
    worst_individuals = [population_list[i][0] for i in range(0, n)]

    # Replace each worst ID with the offspring
    for bad_individual, offspring in zip(worst_individuals, offspring_dictionary):
        population_dictionary[bad_individual] = offspring_dictionary[offspring]
