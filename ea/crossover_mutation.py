"""
This file contains mutation strategies
"""
import random
import numpy as np


def choose_parents(genotype_dictionary):
    """
    :param genotype_dictionary: A dictionary containing the genotypes of offsprings
    :return: Keys of two offspring genotypes to recombine
    """

    key_list = random.sample(range(len(genotype_dictionary)), 2)

    return key_list


def one_point_crossover(genotype_dictionary, key_list):
    """
    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the offspring
    :return: The whole dictionary with mutated genotypes
    """

    offspring_1 = genotype_dictionary[key_list[0]]
    offspring_2 = genotype_dictionary[key_list[1]]

    point = random.randint(1, len(offspring_1[0]) - 1)

    off1_half_copy = offspring_1[0][point:]
    off2_half_copy = offspring_2[0][point:]

    offspring_1[0] = offspring_1[0][:point]+off2_half_copy
    offspring_2[0] = offspring_2[0][:point]+off1_half_copy

    return


def uniform_crossover(genotype_dictionary, key_list):
    """
    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the offspring
    :return: The whole dictionary with mutated genotypes
    """

    offspring_1 = genotype_dictionary[key_list[0]]
    offspring_2 = genotype_dictionary[key_list[1]]

    new_genotype_1, new_genotype_2 = [], []

    for i in range(len(offspring_1[0])):
        rng = random.randint(0,99)
        if rng < 50:
            new_genotype_1 = new_genotype_1 + offspring_1[0][i]
            new_genotype_2 = new_genotype_2 + offspring_2[0][i]
        else:
            new_genotype_1 = new_genotype_1 + offspring_2[0][i]
            new_genotype_2 = new_genotype_2 + offspring_1[0][i]

    offspring_1[0], offspring_2[0] = new_genotype_1, new_genotype_2

    return


def arithmetic_crossover(genotype_dictionary, key_list, integers=True):
    """
    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the offspring
    """

    offspring_1 = genotype_dictionary[key_list[0]]
    offspring_2 = genotype_dictionary[key_list[1]]

    genotype_1 = np.array(offspring_1[0])
    genotype_2 = np.array(offspring_2[0])

    genotype_sum = genotype_1 + genotype_2
    if integers:
        offspring_1[0] = [int(gene/2) for gene in genotype_sum]
        offspring_2[0] = [int(gene/2) for gene in genotype_sum]
    else:
        offspring_1[0] = [gene/2 for gene in genotype_sum]
        offspring_2[0] = [gene/2 for gene in genotype_sum]

    return

def mutation(genotype_dictionary, target_offspring, mutation_range = 1, mutation_rate = 2):
    """
    :param genotype_dictionary: The dictionary containing all genotypes
    :param target_offspring: A key in the genotype dictionary indicating the offspring to be mutated
    """

    offspring = genotype_dictionary[target_offspring]
    steps = 20*mutation_range
    deltas = [mutation_range*(-1) + (step * 0.1) for step in range(0, steps)]

    for gene in offspring[0]:
        rng = random.randint(0, 99)
        if rng < mutation_rate:
            rng2 = random.choice(["+", "-"])
            if rng2 == "+":
                gene = gene + random.choice(deltas)

    return
