"""
Autonomous Robotic Systems (2223-KEN4114)
Assignment 2: Robot Simulator

@authors:
Diyon Wickrameratne (i6176139)
Luca Forte (I6330944)
Olmo Denegri (i6333396)
Florent Didascalou (i6337071)
"""
import math
import pygame
import copy

from robot_simulator.robot import Robot
from ann.Ann import Ann
from shapely.geometry import Point
from ea.robot_evaluation import ann_structure
from robot_simulator.robot import robot_radius
from robot_simulator.robot import robot_border_size


is_running = True
window_size = (900, 800)

# Display the left and right wheel velocities a certain distance (pixels) away from the centre of the robot
velocity_display_distance = 20

# Display the sensor's sensed distance a certain distance (pixels) away from the centre of the robot
sensor_display_distance = 70

# By how much should the velocity of the wheels increase/decrease
velocity_change = 1


def run(robot, room):
    """
    This runs the GUI for the robot simulator

    :param room: Room list
    :param robot: A robot object
    :return: None
    """

    pygame.init()

    # Create display
    pygame.display.set_caption("Robot Simulator")
    window_surface = pygame.display.set_mode(window_size)

    # Create background
    background = pygame.Surface(window_size)
    background.fill(pygame.Color('#FFFFFF'))

    clock = pygame.time.Clock()

    walls = room[0]
    dust = room[1]

    global is_running
    while is_running:

        # Event handler
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                is_running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                print(pygame.mouse.get_pos())

            if event.type == pygame.KEYDOWN:

                # +ve increment of left wheel speed
                if event.key == pygame.K_w:
                    robot.v_l += velocity_change

                # -ve increment of left wheel speed
                if event.key == pygame.K_s:
                    robot.v_l -= velocity_change

                # +ve increment of right wheel speed
                if event.key == pygame.K_o:
                    robot.v_r += velocity_change

                # +ve increment of right wheel speed
                if event.key == pygame.K_l:
                    robot.v_r -= velocity_change

                # Both wheel speeds set to 0
                if event.key == pygame.K_x:
                    robot.v_l = 0
                    robot.v_r = 0

                # +ve increment of both wheel speeds
                if event.key == pygame.K_t:
                    robot.v_l += velocity_change
                    robot.v_r += velocity_change

                # -ve increment of both wheel speeds
                if event.key == pygame.K_g:
                    robot.v_l -= velocity_change
                    robot.v_r -= velocity_change

        robot.update_position()
        robot_centre = Point(robot.pos)

        # Draw background, robot and walls
        window_surface.blit(background, (0, 0))

        pygame.draw.circle(window_surface, "#000000", robot.pos, robot_radius, width=robot_border_size)

        robot_line_end = (robot.pos[0] + robot_radius * math.cos(math.radians(robot.theta))
                          - robot_border_size * math.cos(math.radians(robot.theta)),

                          robot.pos[1] + robot_radius * math.sin(math.radians(robot.theta))
                          - robot_border_size * math.sin(math.radians(robot.theta)))

        pygame.draw.line(window_surface, "#000000", robot.pos, robot_line_end, width=2)

        for wall in walls:
            pygame.draw.line(window_surface, "#000000", wall.coords[0], wall.coords[1], width=2)

        font = pygame.font.Font(None, 20)

        # Plot the dust particles. If the dust particle lies within the circle, i.e. the dust particle's centre
        # is within the radius of the robot, it is collected by the robot and not plotted
        to_remove = []
        for particle in dust:
            x, y = particle.x, particle.y
            particle_pos = Point((x, y))

            if robot_centre.distance(particle_pos) <= robot_radius:
                to_remove.append(particle)
            else:
                pygame.draw.circle(window_surface, "#FF0000", (particle.x, particle.y), 5)

        for particle in to_remove:
            dust.remove(particle)

        robot.dust += len(to_remove)

        # Plot the dust particle counter
        dust_text = font.render(str(robot.dust), True, "#FF0000")
        dust_rectangle = dust_text.get_rect()
        dust_rectangle.center = (
            robot.pos[0] + velocity_display_distance * math.cos(math.radians(robot.theta - 180)),
            robot.pos[1] + velocity_display_distance * math.sin(math.radians(robot.theta - 180))
        )
        window_surface.blit(dust_text, dust_rectangle)

        # Plot the wheel velocities
        v_l_text = font.render(str(robot.v_l), True, "#000000")
        v_r_text = font.render(str(robot.v_r), True, "#000000")

        v_l_rectangle = v_l_text.get_rect()
        v_l_rectangle.center = (
            robot.pos[0] + velocity_display_distance * math.cos(math.radians(robot.theta - 90)),
            robot.pos[1] + velocity_display_distance * math.sin(math.radians(robot.theta - 90))
        )

        v_r_rectangle = v_r_text.get_rect()
        v_r_rectangle.center = (
            robot.pos[0] + velocity_display_distance * math.cos(math.radians(robot.theta + 90)),
            robot.pos[1] + velocity_display_distance * math.sin(math.radians(robot.theta + 90))
        )

        window_surface.blit(v_l_text, v_l_rectangle)
        window_surface.blit(v_r_text, v_r_rectangle)

        # Draw sensors. Each element in the list is a sensor. Each sensor is a tuple (angle, sensor value)
        sensors = [(sensor.angle, sensor.sense_distance2(), sensor.p1, sensor.p2) for sensor in robot.sensors]

        for sensor in sensors:

            # This is temporary, just to show the lines of the sensor's detection range
            # pygame.draw.line(window_surface, "#000000", sensor[2], sensor[3], width=2)

            sensor_distance = font.render(str(round(sensor[1])), True, "#000000")

            sensor_rectangle = sensor_distance.get_rect()
            sensor_rectangle.center = (
                robot.pos[0] + sensor_display_distance * math.cos(math.radians(robot.theta - 90 - sensor[0])),
                robot.pos[1] + sensor_display_distance * math.sin(math.radians(robot.theta - 90 - sensor[0]))
            )

            window_surface.blit(sensor_distance, sensor_rectangle)

        clock.tick(60)
        pygame.display.update()


def run_genotype(genotype, room, initial_pos):
    
    pygame.init()

    # Create display
    pygame.display.set_caption("Robot Simulator")
    window_surface = pygame.display.set_mode(window_size)

    # Create background
    background = pygame.Surface(window_size)
    background.fill(pygame.Color('#FFFFFF'))

    clock = pygame.time.Clock()

    new_room = copy.deepcopy(room)
    new_room_2 = copy.deepcopy(room)
    dust = new_room_2[1]
    
    brain = Ann(ann_structure, genotype)
    body = Robot(1, initial_pos, new_room, n_sensors=12)
    delta_t = 4
    max_vel = 20

    for i in range(200):

        if i % delta_t == 0:

            sensor_data = [sensor.sense_distance2() for sensor in body.sensors]
            vel = brain.feedforward(sensor_data, brain.weights)

            if vel[0] > 0:
                body.set_vel_left(min(vel[0], max_vel))
            else:
                body.set_vel_left(max(vel[0], -max_vel))
            if vel[1] > 0:
                body.set_vel_right(min(vel[1], max_vel))
            else:
                body.set_vel_right(max(vel[1], -max_vel))

        body.update_position()

        # Draw background, robot and walls
        window_surface.blit(background, (0, 0))
        
        pygame.draw.circle(window_surface, "#000000", body.pos, robot_radius, width=robot_border_size)

        robot_line_end = (body.pos[0] + robot_radius * math.cos(math.radians(body.theta))
                          - robot_border_size * math.cos(math.radians(body.theta)),

                          body.pos[1] + robot_radius * math.sin(math.radians(body.theta))
                          - robot_border_size * math.sin(math.radians(body.theta)))

        pygame.draw.line(window_surface, "#000000", body.pos, robot_line_end, width=2)
        
        for wall in room[0]:
            pygame.draw.line(window_surface, "#000000", wall.coords[0], wall.coords[1], width=2)

        font = pygame.font.Font(None, 20)

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

        for j in range(len(dust)):
            for k in range(len(dust[0])):
                if dust[j][k] == 1:
                    pygame.draw.circle(window_surface, "#FF0000", (k, j), 2)

        v_l_text = font.render(str(body.v_l), True, "#000000")
        v_r_text = font.render(str(body.v_r), True, "#000000")

        v_l_rectangle = v_l_text.get_rect()
        v_l_rectangle.center = (
            body.pos[0] + velocity_display_distance * math.cos(math.radians(body.theta - 90)),
            body.pos[1] + velocity_display_distance * math.sin(math.radians(body.theta - 90))
        )

        v_r_rectangle = v_r_text.get_rect()
        v_r_rectangle.center = (
            body.pos[0] + velocity_display_distance * math.cos(math.radians(body.theta + 90)),
            body.pos[1] + velocity_display_distance * math.sin(math.radians(body.theta + 90))
        )

        window_surface.blit(v_l_text, v_l_rectangle)
        window_surface.blit(v_r_text, v_r_rectangle)

        # Draw sensors. Each element in the list is a sensor. Each sensor is a tuple (angle, sensor value)
        sensors = [(sensor.angle, sensor.sense_distance2(), sensor.p1, sensor.p2) for sensor in body.sensors]

        for sensor in sensors:

            # This is temporary, just to show the lines of the sensor's detection range
            # pygame.draw.line(window_surface, "#000000", sensor[2], sensor[3], width=2)

            sensor_distance = font.render(str(round(sensor[1])), True, "#000000")

            sensor_rectangle = sensor_distance.get_rect()
            sensor_rectangle.center = (
                body.pos[0] + sensor_display_distance * math.cos(math.radians(body.theta - 90 - sensor[0])),
                body.pos[1] + sensor_display_distance * math.sin(math.radians(body.theta - 90 - sensor[0]))
            )

            window_surface.blit(sensor_distance, sensor_rectangle)

        clock.tick(60)
        pygame.display.update()
