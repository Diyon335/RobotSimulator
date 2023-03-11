from shapely import Point, LineString
import numpy as np

from robot_simulator.robot import Robot
from robot_simulator.rooms import room_1
from ann.Ann import Ann


sigmoid_stretch = 0.5

itterations = 1000
ann_structure = [12, 3, 2]
walls = room_1[0]
dust = room_1[1]
total_dust = len(dust)

initial_pos = (40, 40)
robot_radius = 52
max_vel = 40

delta_t = 20


def sigmoid(x):
    return (2 / (1 + np.exp(- sigmoid_stretch * x))) - 1


def evaluate_genotype(genotype, ind):
    brain = Ann(ann_structure, genotype)
    body = Robot(ind, initial_pos, room_1, n_sensors=12)
    # weight_lists = brain.create_weights_lists()
    collision_counter = 0

    for i in range(itterations):
        sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
        if i % delta_t == 0:
            vel = brain.feedforward(sensor_data, brain.weights)
            body.set_vel_left(min(vel[0], max_vel))
            body.set_vel_right(min(vel[1], max_vel))

        body.update_position()
        robot_centre = Point(body.pos)

        if min(sensor_data) < 3:
            collision_counter += 1
        
        to_remove = []
        global dust
        for particle in dust:
            x, y = particle.x, particle.y
            particle_pos = Point((x, y))

            if robot_centre.distance(particle_pos) <= robot_radius:
                to_remove.append(particle)

        for particle in to_remove:
            dust.remove(particle)

        body.dust += len(to_remove)

    return (body.dust / total_dust) - sigmoid(collision_counter)