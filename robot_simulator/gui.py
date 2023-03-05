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
from shapely.geometry import LineString

is_running = True
window_size = (900, 800)

robot_radius = 52

# Do not change. This is the width (in pixels) of the robot's outline
robot_border_size = 2

walls = [
    # Standard walls
    LineString([(110, 90), (810, 90)]),
    LineString([(110, 90), (110, 710)]),
    LineString([(110, 710), (810, 710)]),
    LineString([(810, 710), (810, 90)])

    # Upward slope
    # LineString([(400, 300), (810, 90)])

    # Downward slope
    # LineString([(400, 300),(0, 0)])

    # Outward corner
    # LineString([(450, 600), (200, 710)]),
    # LineString([(450, 600), (600, 710)])

    # Outward corner
    # LineString([(450, 600), (200, 600)]),
    # LineString([(450, 600), (450, 800)])

    # Narrow corner
    # LineString([(450, 600), (150, 90)]),
    # LineString([(450, 600), (600, 90)])
]

# Display the left and right wheel velocities a certain distance (pixels) away from the centre of the robot
velocity_display_distance = 20

# Display the sensor's sensed distance a certain distance (pixels) away from the centre of the robot
sensor_display_distance = 70

# By how much should the velocity of the wheels increase/decrease
velocity_change = 2


def run(robot):
    """
    This runs the GUI for the robot simulator

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

    global is_running
    while is_running:

        # Event handler
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                is_running = False

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

        # Draw velocity rectangles
        font = pygame.font.Font(None, 20)

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
