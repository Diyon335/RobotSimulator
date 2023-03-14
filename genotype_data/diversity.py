from itertools import combinations
import matplotlib.pyplot as plt

f = open("rooms14_100gens_6k_5mr_30pop_25off.txt", "r")
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

div1 = diversity[:101]
div2 = diversity[101:202]
div3 = diversity[202:303]

fin_div = []
i = 0
while i < len(div1):
    fin_div.append(sum([div1[i], div2[i], div3[i]])/3)
    i += 1

plt.plot(fin_div)
plt.xlabel("Generations")
plt.ylabel("Diversity")
plt.title("100 Generations, 3 Runs")
plt.show()
