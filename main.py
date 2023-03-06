"""
Autonomous Robotic Systems (2223-KEN4114)
Assignment 2: Robot Simulator
@authors:
Diyon Wickrameratne (i6176139)
Luca Forte (I6330944)
Olmo Denegri (i6333396)
Florent Didascalou (i6337071)
"""
import matplotlib.pyplot as plt
import numpy as np
import time

from robot_simulator import gui
from robot_simulator.gui import walls
from robot_simulator.robot import Robot

from ea.evolutionary_algorithm import run_algorithm, animate_evolution


# Where to place the robot at the start
robot_start = (450, 400)
run_experiments = False

if __name__ == '__main__':

    # robot = Robot(1, robot_start, walls, n_sensors=12)
    # gui.run(robot)

    best, itter = run_algorithm(8)

    print("best fitness: " + str(best))
    print("Itterations untill convergence: " + str(itter))
    print("DONE!!!")
    animate_evolution()

    
    if run_experiments:
        # parameter = "offsprings_per_generations"
        # parameter = "mutation_rate"

        # I suggest using the following parameter sets:
        # [3, 4, 5, 6, 10, 15, 20] for tournament_k
        # [2, 5, 10, 15, 20, 30, 40, 50] for offsprings_per_generations
        # [0, 3, 5, 10, 15, 20] for mutation_rate

        benchmark = 'Rosenbrock'
        #benchmark = 'Rastrigin'

        #parameter = 'Mutation_rate'
        #param_set = np.arange(0, 20.5, 0.5)
        
        parameter = 'Offspring_per_generation'
        param_set = np.arange(2,51,1)

        #parameter = 'Tournament_k'
        #param_set = np.arange(2,21,1)
        avg_best = []
        avg_itter_count = []

        for param in param_set:
            hist_best = []
            hist_itter_count = []
            for i in range(30):
                run_best, run_itter = run_algorithm(param)
                hist_best.append(run_best)
                hist_itter_count.append(run_itter)
            avg_best.append(sum(hist_best) / 30)
            avg_itter_count.append(sum(hist_itter_count) / 30)
            print(param)

        print(avg_itter_count)
        print(avg_best)

        fig_conv = plt.figure()
        plt.plot(param_set, avg_itter_count, 'ob')
        fig_conv.suptitle(f'Comparing convergence of EA on {benchmark} with varying {parameter}')
        plt.xlabel(parameter)
        plt.ylabel('Average Itteration Count')
        fig_conv.savefig(f'{parameter}_{benchmark[0:3]}_convergence.jpg')
        plt.clf()

        fig_best = plt.figure()
        plt.plot(param_set, avg_best, 'ob')
        fig_best.suptitle(f'Comparing performance of EA on {benchmark} with varying {parameter}')
        plt.ylim(top=0)
        plt.xlabel(parameter)
        plt.ylabel('Average Best Fitness')
        fig_best.savefig(f'{parameter}_{benchmark[0:3]}_fitness.jpg')
