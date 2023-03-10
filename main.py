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

from ann.Ann import Ann
from robot_simulator import gui
from robot_simulator.gui import walls
from robot_simulator.robot import Robot

from ea.evolutionary_algorithm import run_algorithm, animate_evolution


# Where to place the robot at the start
robot_start = (450, 400)


if __name__ == '__main__':

    # robot = Robot(1, robot_start, walls, n_sensors=12)
    # gui.run(robot)

    #run_algorithm()
    #animate_evolution()

    ann = Ann([4,3,2], [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34])
    weights_lists = ann.create_weights_lists()
    print(weights_lists)
    output = ann.feedforward([1,2,3,4], weights_lists)