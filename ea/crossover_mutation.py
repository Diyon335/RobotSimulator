"""
This file contains mutation strategies
"""
import random


def choose_parents(genotype_dictionary):

    '''
    :param genotype_dictionary: The dictionary containing all genotypes
    :return: The two parents keys to recombine
    '''

    key_list = random.sample(range(len(genotype_dictionary)), 2)

    return key_list

def one_point_crossover(genotype_dictionary, key_list):

    '''
    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the parents
    :return: The whole dictionary with mutated genotypes
    '''

    point = random.randint(1, len(genotype_dictionary) - 1)

    for i in range(len(genotype_dictionary[key_list[0]][0])):
        if i < point:
            genotype_dictionary[key_list[0]][0][i] = genotype_dictionary[key_list[1]][0][i]
        else:
            genotype_dictionary[key_list[1]][0][i] = genotype_dictionary[key_list[0]][0][i]



    return genotype_dictionary

def uniform_crossover(genotype_dictionary, key_list):

    '''

    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the parents
    :return: The whole dictionary with mutated genotypes
    '''

    return genotype_dictionary


def arithmetic_crossover(genotype_dictionary, key_list):

    '''

    :param genotype_dictionary: The dictionary containing all genotypes
    :param key_list: A list containing the keys of the parents
    :return: The whole dictionary with mutated genotypes
    '''

    return genotype_dictionary