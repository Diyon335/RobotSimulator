"""
This file contains the encoding strategies for the EA
"""
import random


def real_number_encoding(size, min_value, max_value, integers=True):
    """
    Encodes a genotype with whole or decimal real numbers

    :param min_value: An integer that sets the minimum of the range for which random numbers can be generated
    :param integers: A boolean indicating whether integer numbers are desired. If not, encoding will contain decimals
    :param max_value: An integer that sets the maximum of the range for which random numbers can be generated
    :param size: An integer that indicates the size of the genotype
    :return: Returns an array of integer or decimal numbers
    """

    return [random.randint(min_value, max_value) for _ in range(size)] if integers else \
        [random.uniform(min_value, max_value) for _ in range(size)]

