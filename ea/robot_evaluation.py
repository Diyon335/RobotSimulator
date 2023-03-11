from shapely import Point, LineString
import numpy as np

from robot_simulator.robot import Robot
from robot_simulator.rooms import room_1
from robot_simulator.gui import robot_radius
from ann.Ann import Ann

import time


sigmoid_stretch = 0.5

itterations = 600
ann_structure = [12, 3, 2]
#walls = room_1[0]
#dust = room_1[1]
#total_dust = len(dust)

initial_pos = (450, 400)
robot_radius = 52
max_vel = 40

delta_t = 20


def sigmoid(x):
    return (2 / (1 + np.exp(- sigmoid_stretch * x))) - 1


def evaluate_genotype(genotype, ind, room):
    dust = room[1]
    
    start = time.time()
    brain = Ann(ann_structure, genotype)
    body = Robot(ind, initial_pos, room, n_sensors=12)
    # weight_lists = brain.create_weights_lists()
    collision_counter = 0
    c = 0

    for i in range(itterations):
        # sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
        if i % delta_t == 0:
            sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
            vel = brain.feedforward(sensor_data, brain.weights)
            body.set_vel_left(min(vel[0], max_vel))
            body.set_vel_right(min(vel[1], max_vel))

        if body.update_position():
            collision_counter += 1

        robot_centre = Point(body.pos)

        '''if min(sensor_data) < 3:
            collision_counter += 1'''

        # to_remove = []
        # global dust
        # for particle in dust:

        #     if robot_centre.distance(particle) <= robot_radius:
        #         to_remove.append(particle)

        # # for particle in to_remove:
        # #     dust.remove(particle)

        # body.dust += len(to_remove)

        removed = 0

        x, y = body.pos
        x_min = max(x-robot_radius, 15)
        x_max = min(x+robot_radius, len(dust[0]))
        y_min = max(y-robot_radius, 20)
        y_max = min(y+robot_radius, len(dust))

        for i in range(int(y_min), int(y_max), 1):
            for j in range(int(x_min), int(x_max), 1):
                if dust[i][j] == 1:
                    dust[i][j] = 0
                    removed += 1

        body.dust += removed

    print(time.time()-start)
    # print(c)
    print("done evaluation")

    print(time.time()-start)
    # print(c)
    print("done evaluation")

    return (body.dust / room[2]) - sigmoid(collision_counter)