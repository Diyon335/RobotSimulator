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

    results = testing_routine(parameter_set, parameter, tests)
    print()
    for key in results:
        print(key)
        for elem in results[key]:
            print(elem)
        print()
