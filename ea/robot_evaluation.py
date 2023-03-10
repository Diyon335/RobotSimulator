from robot_simulator.robot import Robot
from ann.Ann import Ann
from shapely import Point, LineString


from robot_simulator.rooms import room_1

itterations = 1000
ann_structure = [16, 2]
walls = room_1[0]
dust = room_1[1]

initial_pos = (40,40)
robot_radius = 52

delta_t = 20

def evaluate_genotype(genotype):
    brain = Ann(ann_structure, genotype)
    body = Robot(genotype, initial_pos, walls, n_sensors=12)
    weight_lists = brain.create_weights_lists()

    for i in range(itterations):
        sensor_data = [sensor.sense_distance2 for sensor in body.sensors]
        if i % delta_t == 0:
            vel = brain.feedforward(sensor_data, weight_lists)
            body.set_vel_left(vel[0])
            body.set_vel_right(vel[1])
        
        
        body.update_position()
        robot_centre = Point(body.pos)
        
        to_remove = []
        for particle in dust:
            x, y = particle.x, particle.y
            particle_pos = Point((x, y))

            if robot_centre.distance(particle_pos) <= robot_radius:
                to_remove.append(particle)
           

        for particle in to_remove:
            dust.remove(particle)

        body.dust += len(to_remove)
    

    return None
