from shapely import Point, LineString
import numpy as np

from robot_simulator.robot import Robot
from robot_simulator.rooms import room_1
from robot_simulator.robot import robot_radius
from ann.Ann import Ann


import time
import copy


sigmoid_stretch = 0.5

itterations = 600
ann_structure = [12, 3, 2]

initial_pos = (450, 400)
max_vel = 20

delta_t = 6


def sigmoid(x):
    return (2 / (1 + np.exp(- sigmoid_stretch * x))) - 1


def sensor_squash(t):
    A = 100
    alpha = 0.1
    tau = 1
    return A + (alpha*A -A)*(1-np.exp(-t/tau))


def evaluate_genotype(genotype, ind, room):

    new_room = copy.deepcopy(room)
    dust = new_room[1]
    
    start = time.time()

    brain = Ann(ann_structure, genotype)
    body = Robot(ind, initial_pos, room, n_sensors=12)
    collision_counter = 0
    current_collision = False

    for i in range(itterations):

        if i % delta_t == 0:
            sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
            sensor_data = [sensor_squash(reading) for reading in sensor_data]
            vel = brain.feedforward(sensor_data, brain.weights)
            if vel[0] > 0:
                body.set_vel_left(min(vel[0], max_vel))
            else:
                body.set_vel_left(max(vel[0], -max_vel))
            if vel[1] > 0:
                body.set_vel_right(min(vel[1], max_vel))
            else:
                body.set_vel_right(max(vel[1], -max_vel))

        # sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
        # # print(sensor_data)
        # vel = brain.feedforward(sensor_data, brain.weights)
        # if vel[0] > 0:
        #     body.set_vel_left(min(vel[0], max_vel))
        # else:
        #     body.set_vel_left(max(vel[0], -max_vel))
        # if vel[1] > 0:
        #     body.set_vel_right(min(vel[1], max_vel))
        # else:
        #     body.set_vel_right(max(vel[1], -max_vel))

        if body.update_position():
            if not current_collision:
                collision_counter += 1
                current_collision = True
        else:
            current_collision = False

        # robot_centre = Point(body.pos)
        #
        # to_remove = []
        # for particle in dust:
        #
        #     if robot_centre.distance(particle) <= robot_radius:
        #         to_remove.append(particle)
        #
        # for particle in to_remove:
        #     dust.remove(particle)
        #
        # body.dust += len(to_remove)

        removed = 0

        x, y = body.pos
        x, y = int(x), int(y)

        x_min = max(x-robot_radius, 15)
        x_max = min(x+robot_radius, len(dust[0]))
        y_min = max(y-robot_radius, 20)
        y_max = min(y+robot_radius, len(dust))

        for j in range(y_min, y_max):
            for k in range(x_min, x_max):
                if dust[j][k] == 1:
                    dust[j][k] = 0
                    removed += 1

        body.dust += removed

    print(f"\tEvaluated {ind} in {time.time() - start} seconds")

    dust_left = sum([sum(row) for row in dust])
    dust_removed = room[2] - dust_left

    fitness = (dust_removed / room[2]) - sigmoid(collision_counter)
    return fitness
