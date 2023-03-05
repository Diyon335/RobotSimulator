"""
This file contains selection strategies for the evolutionary algorithm
"""
import random


def tournament_selection(genotype_dictionary, k=5):
    """
    Tournament selection strategy takes a sample, size k, from all the genotypes and returns the best genotype
    of the sample

    :param genotype_dictionary: The dictionary containing all genotypes
    :param k: An integer indicating the sample size
    :return: Returns an integer indicating the best genotype. The integer is the dictionary key of the best genotype
    """

    # Get all the genotype IDs
    genotypes = list(genotype_dictionary.keys())

    # Randomly sample k different IDs
    genotypes_to_sample = random.sample(genotypes, k)

    # For each sampled ID, get the genotype, fitness and phenotype from the genotype dict, and put into a list
    # Each element in the list will be of the form (genotype, fitness, (phenotype))
    sampled_genotypes = [genotype_dictionary[genotype] for genotype in genotypes_to_sample]

    # Gets the best genotype from the sample based on the 1st index (fitness)
    best_genotype = max(sampled_genotypes, key=lambda genotype: genotype[1])

    genotype_values = list(genotype_dictionary.values())

    # Return the ID of the best genotype, which is also just the index within the list of genotype values
    return genotype_values.index(best_genotype)
