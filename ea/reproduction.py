"""
This file contains reproduction strategies
"""

def reproduction_tournament_selection(genotype_dictionary, index):

    '''
    :param genotype_dictionary: The dictionary containing all genotypes
    :param index: The index of the best parent among the k analyzed to copy
    :return: The new dictionary after we added the copy of the best parent
    '''

    genotype_dictionary[len(genotype_dictionary)] = genotype_dictionary[index]

    return genotype_dictionary
