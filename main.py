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

from robot_simulator import gui
from robot_simulator.gui import walls
from robot_simulator.robot import Robot

from ea.evolutionary_algorithm import run_algorithm, animate_evolution, testing_routine


# Where to place the robot at the start
robot_start = (450, 400)


if __name__ == '__main__':

    # robot = Robot(1, robot_start, walls, n_sensors=12)
    # gui.run(robot)

    # run_algorithm()
    # animate_evolution()

    tests = 100

    parameter = "tournament_k"
    # parameter = "offsprings_per_generations"
    # parameter = "mutation_rate"

    # I suggest using the following parameter sets:
    # [3, 4, 5, 6, 10, 15, 20] for tournament_k
    # [2, 5, 10, 15, 20, 30, 40, 50] for offsprings_per_generations
    # [0, 3, 5, 10, 15, 20] for mutation_rate
    parameter_set = [3, 4, 5, 6, 10, 15, 20]

    results = testing_routine(parameter_set, parameter, tests, True)
    print()
    for key in results:
        print(key)
        for elem in results[key]:
            print(elem)
        print()

    for key in parameter_set:
        fig_max = plt.figure()
        x1 = results[key][0]
        x2 = results[key][1]
        line_max = plt.plot(x1, 'g', label='Max_fitness')
        line_avg = plt.plot(x2, 'b', label='Average_fitness')
        plt.title(f'Generation vs. Fitness for {parameter} = {key}')
        plt.legend()
        plt.xlabel('Generation')
        plt.ylabel('Fitness')
        fig_max.savefig(f'tournament_k_{key}.jpg')
        