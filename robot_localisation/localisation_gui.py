import pygame
import numpy as np
from robot_localisation.mobile_robot import robot_radius, robot_border_size
from robot_localisation.room import room


is_running = True
window_size = (900, 800)

velocity_change = 1

# 1 degree
omega_change = 0.017

feature_radius = 10
walls = room[0]
features = room[1]


def run(robot):
    """
    This runs the GUI for the robot localiser

    :param robot: A robot object
    :return: None
    """

    pygame.init()

    # Create display
    pygame.display.set_caption("Robot Localiser")
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

            if event.type == pygame.MOUSEBUTTONDOWN:
                print(pygame.mouse.get_pos())

            if event.type == pygame.KEYDOWN:

                # Increase velocity
                if event.key == pygame.K_w:
                    robot.velocity += velocity_change

                # Decrease velocity
                if event.key == pygame.K_s:
                    robot.velocity -= velocity_change

                # Decrease omega
                if event.key == pygame.K_a:
                    robot.omega -= omega_change

                # Increase omega
                if event.key == pygame.K_d:
                    robot.omega += omega_change

                # Stop robot
                if event.key == pygame.K_x:
                    robot.velocity = 0
                    robot.omega = 0

        robot.update_position()

        # Draw background, robot, walls and features
        window_surface.blit(background, (0, 0))

        pygame.draw.circle(window_surface, "#000000", robot.pos, robot_radius, width=robot_border_size)

        robot_line_end = (robot.pos[0] + robot_radius * np.cos(robot.theta)
                          - robot_border_size * np.cos(robot.theta),

                          robot.pos[1] + robot_radius * np.sin(robot.theta)
                          - robot_border_size * np.sin(robot.theta))

        pygame.draw.line(window_surface, "#000000", robot.pos, robot_line_end, width=2)

        for feature in features:
            pygame.draw.circle(window_surface, "#000000", feature, feature_radius)

            # If robot is close enough, draw green line
            if distance_to_feature(robot.pos, feature) < robot_radius + robot.sensor_range:
                pygame.draw.line(window_surface, "#008000", robot.pos, feature, width=2)

        for wall in walls:
            pygame.draw.line(window_surface, "#000000", wall[0], wall[1], width=2)

        clock.tick(60)
        pygame.display.update()


def distance_to_feature(robot_pos, feature_pos):
    """
    Uses pythagoras theorem to compute distance from robot centre to feature centre

    :param robot_pos: Tuple of robot's pos
    :param feature_pos: Tuple of feature's pos
    :return: Returns a float
    """

    x1, y1 = robot_pos
    x2, y2 = feature_pos

    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
