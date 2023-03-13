from itertools import combinations

f = open("tanh_run.txt", "r")
lines = f.readlines()

diversity = []

genotype = False

for line in lines:
    if genotype:
        diversity[-1].append([float(gene) for gene in line[11:-2].split(", ")])
        if len(diversity[-1]) == 30:
            div = 0
            for geno_pair in combinations(diversity[-1], 2):
                i = 0
                while i < len(geno_pair[0]):
                    div += abs(geno_pair[0][i] - geno_pair[1][i])
                    i += 1
            diversity[-1] = div
        genotype = False
    if "ID" in line:
        genotype = True
        if ": 0" in line:
            diversity.append([])

print(len(diversity))
print(diversity)
