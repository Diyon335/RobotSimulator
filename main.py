"""
Autonomous Robotic Systems (2223-KEN4114)
Assignment 2: Robot Simulator
@authors:
Diyon Wickrameratne (i6176139)
Luca Forte (I6330944)
Olmo Denegri (i6333396)
Florent Didascalou (i6337071)
"""
from robot_simulator.rooms import room_2, room_1
from ea.nn_evolutionary_algorithm import run_algorithm, test_algorithm_with_parameters


# Where to place the robot at the start
robot_start = (450, 400)


if __name__ == '__main__':

    # robot = Robot(1, robot_start, room_2, n_sensors=12)
    # gui.run(robot)

    # run_algorithm(room_2)

    # # animate_evolution()

    # '''ann = Ann([4, 3, 2],
    #           [1, 2, 3, 4, 5, 6, 7, 8, 9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4,
    #            2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2])
    # print(ann.feedforward([1, 2, 3, 4], ann.weights))
    # print(ann.feedforward([5, 6, 7, 8], ann.weights))'''

    test_algorithm_with_parameters(room_2, 5, tests=5)
