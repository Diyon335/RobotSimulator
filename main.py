"""
Autonomous Robotic Systems (2223-KEN4114)
Assignment 2: Robot Simulator

@authors:
Diyon Wickrameratne (i6176139)
Luca Forte (I6330944)
Olmo Denegri (i6333396)
Florent Didascalou (i6337071)
"""

import gui
from gui import walls
from robot import Robot


# Where to place the robot at the start
robot_start = (450, 400)


if __name__ == '__main__':

    robot = Robot(1, robot_start, walls)
    robot.generate_sensors(2)
    gui.run(robot)

