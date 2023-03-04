import random
import numpy as np

pop_size = 50
genotype = []

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

def apply_mutation(tup, mutation_chance, mutation_size = 0.05):
    new_gene = []

    for i in range(len(tup)):
        if random.random() < mutation_chance:
            pos = bool(random.getrandbits(1))
            if pos:
                new_gene.append(tup[0] + (mutation_size * tup[0]))
            else:
                new_gene.append(tup[0] - (mutation_size * tup[0]))
        else:
            new_gene.append(tup[i])
                
    ## TODO modify for longer gene encoding
    return new_gene[0], new_gene[1]

def run_algorithm():
    ## initialization
    genotypes = [(random.uniform(-5, 5), random.uniform(-5, 5)) for i in range(pop_size)]

    population_info = {}

    for i in range(100):
        ## evaluation
        for gene in genotypes:
            population_info[gene] = cost_rosenbrock(gene[0], gene[1])
        #best_genotype = min(population_info, key=population_info.get)


        ## selection: tournament
        offspring = []
        k = 5
        for i in range(50):
            genotypes = list(population_info.keys())
            to_sample = random.sample(genotypes, k)
            sampled = [(genotype, population_info[genotype]) for genotype in to_sample]
            best = min(sampled, key=lambda x: x[1])
            offspring.append(((best[0][0], best[0][1]), best[1]))
        #print(fitness_lst)
        #print("offspring count " + str(len(offspring)))

        ## reproduction
        # TODO Never use dictionaries
        #print(max(population_info.values()))
        population_info.clear()
        counter = 0
        for off in offspring:
            counter += 1
            population_info[off[0]] = off[1]
        
        #print("length of pop")
        #print(len(list(population_info.keys())))
        #print("NEW!")
        #print(population_info)
        #print(max(population_info.values()))

        ## clean up offspring
        clean_offspring = [off[0] for off in offspring]

        ## Crossover - arithmetic
        children = []
        
        
        for off in clean_offspring:
            coparrent = random.choice(clean_offspring)
            child = ((off[0] + coparrent[0]) / 2, (off[1] + coparrent[1]) / 2)
            children.append(child)
        #print(len(children))
        #print(children)

        ## Mutation
        mutation_chance = 0.1
        mutation_size = 0.05
        mutated_children = []
        for child in children:
            mutated_children.append(apply_mutation(child, mutation_chance, mutation_size))
        genotypes = mutated_children
        population_info.clear()

    final_population = []
    for gene in genotypes:
        final_population.append([gene, cost_rosenbrock(gene[0], gene[1])])
    
    print(final_population)
    print(min(final_population, key=lambda x: x[1]))


            


    print()

if __name__ == '__main__':

    run_algorithm()