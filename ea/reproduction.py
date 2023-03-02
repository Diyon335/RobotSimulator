"""
This file contains reproduction strategies
"""


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
