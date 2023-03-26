import pygame
import numpy as np
from robot_localisation.mobile_robot import robot_radius, robot_border_size


is_running = True
window_size = (900, 800)

velocity_change = 0.5

# 1 degree
omega_change = 0.017

feature_radius = 10

line_limit = 1000


def run(robot, room, clear_paths=False):
    """
    This runs the GUI for the robot localiser

    :param clear_paths: Boolean. Indicates whether the drawn paths should be cleared after the path lists have exceeded
    their limit
    :param room: Room list with walls and features
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

    walls = room[0]
    features = room[1]

    predicted_path = []
    robot_path = []
    i = 0

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

        pos, predicted_pos, predicted_cov, _ = robot.update_position()
        predicted_path.append(_)
        robot_path.append(pos)

        # Draw background, robot, walls and features
        window_surface.blit(background, (0, 0))

        pygame.draw.circle(window_surface, "#000000", pos, robot_radius, width=robot_border_size)

        robot_line_end = (robot.pos[0] + robot_radius * np.cos(robot.theta)
                          - robot_border_size * np.cos(robot.theta),

                          robot.pos[1] + robot_radius * np.sin(robot.theta)
                          - robot_border_size * np.sin(robot.theta))

        pygame.draw.line(window_surface, "#000000", robot.pos, robot_line_end, width=2)

        # Draw the estimated covariance ellipse
        rect_centre = (_[0] - predicted_cov[0]/2, _[1] - predicted_cov[1]/2)

        print(f"RECT CENTRE: {rect_centre}")
        print(f"COV: {predicted_cov}")
        pygame.draw.ellipse(window_surface, "#000000", pygame.Rect(rect_centre, predicted_cov), width=2)

        # Draw the predicted path and robot's path
        if i > 1:
            pygame.draw.lines(window_surface, "#000000", False, robot_path, 2)
            pygame.draw.lines(window_surface, "#008000", False, predicted_path, 2)

        # Draw features
        for feature in features:
            pygame.draw.circle(window_surface, "#000000", feature, feature_radius)

            # If robot is close enough, draw green line
            if distance_to_feature(robot.pos, feature) < robot_radius + robot.sensor_range:
                pygame.draw.line(window_surface, "#008000", robot.pos, feature, width=2)

        for wall in walls:
            pygame.draw.line(window_surface, "#000000", wall[0], wall[1], width=2)

        clock.tick(60)
        pygame.display.update()

        i += 1

        # If the predicted path list has more than the set limit, reset i and clear the list
        if i > line_limit:
            i = 0

            if clear_paths:
                predicted_path.clear()
                robot_path.clear()


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
