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


if __name__ == '__main__':

    # robot = Robot(1, robot_start, walls, n_sensors=12)
    # gui.run(robot)

    best, itter = run_algorithm(8)

    print("best fitness: " + str(best))
    print("Itterations untill convergence: " + str(itter))
    print("DONE!!!")
    #animate_evolution()

    mutation_rates = np.arange(0, 100.5, 0.5)
    avg_best = []
    avg_itter_count = []

    for rate in mutation_rates:
        hist_best = []
        hist_itter_count = []
        for i in range(30):
            run_best, run_itter = run_algorithm(rate)
            hist_best.append(run_best)
            hist_itter_count.append(run_itter)
        avg_best.append(sum(hist_best) / 30)
        avg_itter_count.append(sum(hist_itter_count) / 30)
        # time.sleep(1)
        print(rate)

    print(avg_itter_count)