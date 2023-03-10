import random
import numpy as np
output = [0,0]
class Ann:

    '''Neural network class. Each robot will have his own neural network instance.'''



    def __init__(self, layers, weights):

        '''
        :param input: The input values from sensors.
        In this example I use a list with 5 input values since I used 5 neurons in the first layer,
        but we can also use multiple input values for each neuron
        :param layers: The architecture of the network
        :param weights: The weights, which are basically the genotype
        '''
        self.input = input
        self.layers = layers
        self.weigths = weights

    def create_weights_lists(self):

        '''
        This method creates different weights lists given the whole genotype.
        Each list represents the weights for a specific layer.
        :return: Different weights lists, the amount of lists is equal to the number of layers - 1
        '''
        weights_lists = []
        i = 0
        weights_lists.append(self.weigths[:self.layers[0] * self.layers[0] + self.layers[0] * len(output)])
        self.weigths = self.weigths[self.layers[0] * self.layers[0] + self.layers[0] * len(output):]
        while i < len(self.layers) - 1:
            weights_lists.append(self.weigths[:self.layers[i] * self.layers[i + 1]])
            self.weigths = self.weigths[self.layers[i] * self.layers[i + 1]:]
            i += 1
        return weights_lists

    def split_list(self, weights_list, n):

        '''
        Given the lists of weights for one layer, this method split it into different lists,
        each of which refers to a particular neuron that has to be computed in the next layer
        :param weights_list: One of the previously calculated list of weights
        :param n: The number that indicates how many parts the list should be divided into
        :return: n different lists of equal lenght
        '''

        weights_array = np.array(weights_list)
        weights_array = np.split(weights_array, n)
        matrix_weights = np.array(weights_array)
        return matrix_weights

    def relu(self, Z):

        '''
        Just the implementation of the relu actvation funtion
        :param Z: Tne value of the neuron
        :return: 0 if the neuron is turned off, Z otherwise
        '''
        return np.maximum(0, Z)

    def feedforward(self, input, weights_lists, l=[]):

        global output
        real_input = input + output
        matrix = [real_input]
        for i in range (0, self.layers[0] - 1):
            matrix.append(real_input)
        '''
        Feedforward routine implementation
        :param weights_lists: All the lists of weights previously computed
        :return: The output of the network, which are the two wheel velocities
        '''
        i = 1
        weights = self.split_list(weights_lists[0], self.layers[0])
        layer = self.relu(np.dot(matrix, weights.transpose()))
        while i < len(self.layers):
            weights = self.split_list(weights_lists[i], self.layers[i])
            layer = self.relu(np.dot(layer, weights.transpose()))
            i += 1
        output = [layer[0][0], layer[0][1]]
        print(output)
        return layer