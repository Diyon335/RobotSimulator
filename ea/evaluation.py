"""
This file contains all evaluation strategies
"""
import numpy as np


def get_xy_phenotype(genotype):
    """
    :param genotype: A list indicating the genotype
    :return: Returns the phenotype of the genotype, where the x is defined as the average
            of the first half of the real-valued genotype and the y is defined as the average of
            the second half of the real-valued genotype array

    """
    first_half = int(len(genotype)/2)
    second_half = len(genotype) - first_half
    x = sum(genotype[:first_half])/first_half
    y = sum(genotype[second_half:])/second_half

    return x, y


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