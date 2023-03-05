"""
This file contains mutation strategies
"""
import random
import numpy as np


def choose_pair(offspring_dictionary):
    """
    :param offspring_dictionary: A dictionary containing the genotypes of offsprings
    :return: Keys of two offspring genotypes to recombine
    """

    key_list = random.sample(range(len(offspring_dictionary)), 2)

    return key_list


def one_point_crossover(offspring_dictionary, key_list):
    """
    :param offspring_dictionary: The dictionary containing genotypes of offsprings
    :param key_list: A list containing the keys of the offspring to recombine
    :return: none
    """

    offspring_1 = offspring_dictionary[key_list[0]]
    offspring_2 = offspring_dictionary[key_list[1]]

    point = random.randint(1, len(offspring_1[0]) - 1)

    off1_half_copy = offspring_1[0][point:]
    off2_half_copy = offspring_2[0][point:]

    offspring_1[0] = offspring_1[0][:point]+off2_half_copy
    offspring_2[0] = offspring_2[0][:point]+off1_half_copy


def uniform_crossover(offspring_dictionary, key_list):
    """
    :param offspring_dictionary: The dictionary containing genotypes of offsprings
    :param key_list: A list containing the keys of the offspring to recombine
    :return: none
    """

    offspring_1 = offspring_dictionary[key_list[0]]
    offspring_2 = offspring_dictionary[key_list[1]]

    new_genotype_1, new_genotype_2 = [], []

    for i in range(len(offspring_1[0])):
        rng = random.randint(0,99)
        if rng < 50:
            new_genotype_1 = new_genotype_1 + [offspring_1[0][i]]
            new_genotype_2 = new_genotype_2 + [offspring_2[0][i]]
        else:
            new_genotype_1 = new_genotype_1 + [offspring_2[0][i]]
            new_genotype_2 = new_genotype_2 + [offspring_1[0][i]]

    offspring_1[0], offspring_2[0] = new_genotype_1, new_genotype_2


def arithmetic_crossover(offspring_dictionary, key_list, integers=True):
    """
    :param offspring_dictionary: The dictionary containing genotypes of offsprings
    :param key_list: A list containing the keys of the offspring to recombine
    :param integers: A boolean determining whether genes should be integers after arithmetic computation
    :return: none
    """

    offspring_1 = offspring_dictionary[key_list[0]]
    offspring_2 = offspring_dictionary[key_list[1]]

    genotype_1 = np.array(offspring_1[0])
    genotype_2 = np.array(offspring_2[0])

    genotype_sum = genotype_1 + genotype_2
    if integers:
        offspring_1[0] = [int(gene/2) for gene in genotype_sum]
        offspring_2[0] = [int(gene/2) for gene in genotype_sum]
    else:
        offspring_1[0] = [gene/2 for gene in genotype_sum]
        offspring_2[0] = [gene/2 for gene in genotype_sum]


def mutation(offspring_dictionary, target_offspring, mutation_rate, mutation_range=5):
    """
    :param offspring_dictionary: The dictionary containing all offsprings
    :param target_offspring: A key in the offspring dictionary indicating the offspring to be mutated
    :param mutation_rate: Chance of ever individual to mutate
    :param mutation_range: by how much can a real-value gene change
    """

    steps = 20*mutation_range
    deltas = [mutation_range*(-1) + (step * 0.1) for step in range(0, steps)]

    for i in range(len(offspring_dictionary[target_offspring][0])):
        rng = random.randint(0, 99)
        if rng < mutation_rate:
            offspring_dictionary[target_offspring][0][i] += random.choice(deltas)
