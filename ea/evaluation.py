"""
This file contains all evaluation strategies
"""

def evaluate(genotype_dictionary):

    '''
    :param genotype_dictionary: The genotype dictionary
    :return: The genotype dictionary with the evaluation calculation

    We need to define how to go from the genotype to a pair of coordinates for the two functions
    In this function the evaluation of each element (initially set to 0) will be modified
    '''


    return genotype_dictionary

def cost_rosenbrock(x, y):
    """
    The Rosenbrock cost function

    :param x: The x-coord of the particle
    :param y: The y-coord of the particle
    :return: Returns an integer indicating the cost
    """

    a = 0
    b = 100
    return (a - x) ** 2 + (b * (y - x ** 2) ** 2)



def cost_rastrigin(x, y, n=2):
    """
    The Rastrigin cost function

    :param x: The x-coord of the particle
    :param y: The y-coord of the particle
    :param n: Dimension of the space
    :return: Returns an integer indicating the cost
    """

    vector = [x, y]
    X = x
    Y = y
    summation = 0
    for j in range(n):
        summation += vector[j] ** 2 - (10 * np.cos(2 * np.pi * vector[j] ** 2))

    # return (10 * n) + summation
    return (X ** 2 - 10 * np.cos(2 * np.pi * X)) + \
           (Y ** 2 - 10 * np.cos(2 * np.pi * Y)) + 20