import math

import pygame
from sympy import Point, Line

is_running = True
window_size = (900, 800)

change_robot_size = False
new_robot_size = (107, 107)

walls = [
    Line(Point(110, 90), Point(810, 90)),
    Line(Point(110, 90), Point(110, 710)),
    Line(Point(110, 710), Point(810, 710)),
    Line(Point(810, 710), Point(810, 90))
]

# Display the left and right wheel velocities a certain distance (pixels) away from the centre of the robot
velocity_display_distance = 20

# Display the sensor's sensed distance a certain distance (pixels) away from the centre of the robot
sensor_display_distance = 60

# By how much should the velocity of the wheels increase/decrease
velocity_change = 1

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

    robot_image = pygame.image.load("robot.png")

    # Default size for the robot is 107x107 pixels. If this needs to be changed, set change_robot_size = True and
    # update new_robot_size
    if change_robot_size:
        robot_image = pygame.transform.scale(robot_image, new_robot_size)

    global is_running
    while is_running:

        # Event handler
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                is_running = False

            # TODO: Change this event to listen to when the robot changes direction to update its heading accordingly
            # TODO: Might not actually be needed. Depending on the key press, the velocities will be changed
            # TODO: so just update the position of the robot after all these if statements
            if event.type == pygame.MOUSEBUTTONDOWN:
                robot.theta += 90
                robot_image = pygame.transform.rotate(robot_image, 60)

            # TODO: Change individual events to their respective functions. For now it's just test stuff
            if event.type == pygame.KEYDOWN:

                # +ve increment of left wheel speed
                if event.key == pygame.K_w:
                    robot.pos = (robot.pos[0], robot.pos[1] - 10)
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

        # TODO: Update robot's pos here
        # robot.update_position()

        # Draw background, robot and walls
        window_surface.blit(background, (0, 0))
        window_surface.blit(robot_image, (robot.pos[0] - new_robot_size[0]/2, robot.pos[1] - new_robot_size[1]/2))

        for wall in walls:
            pygame.draw.line(window_surface, "#000000", wall.p1, wall.p2, width=2)

        # Draw velocity rectangles
        font = pygame.font.Font(None, 20)

        # TODO: Replace l and r with v_l and v_r
        v_l_text = font.render("l", True, "#000000")
        v_r_text = font.render("r", True, "#000000")

        v_l_rectangle = v_l_text.get_rect()
        v_l_rectangle.center = (
            robot.pos[0] + velocity_display_distance * math.cos(math.radians(90 + robot.theta)),
            robot.pos[1] - velocity_display_distance * math.sin(math.radians(90 + robot.theta))
        )

        v_r_rectangle = v_r_text.get_rect()
        v_r_rectangle.center = (
            robot.pos[0] + velocity_display_distance * math.cos(math.radians(robot.theta - 90)),
            robot.pos[1] - velocity_display_distance * math.sin(math.radians(robot.theta - 90))
        )

        window_surface.blit(v_l_text, v_l_rectangle)
        window_surface.blit(v_r_text, v_r_rectangle)

        """This is just a test, this won't be here at the end"""
        robot.sensors[0].sense_distance()

        # TODO: Replace with actual sensor angles and distances
        # Draw sensors. Each element in the list is a sensor. Each sensor is a tuple (angle, sensor value)
        sensors = [(0, "1"), (90, "2"), (180, "3")]

        for sensor in sensors:
            sensor_distance = font.render(sensor[1], True, "#000000")

            sensor_rectangle = sensor_distance.get_rect()
            sensor_rectangle.center = (
                robot.pos[0] + sensor_display_distance * math.cos(math.radians(90 - sensor[0] - robot.theta)),
                robot.pos[1] + sensor_display_distance * math.sin(math.radians(90 - sensor[0] - robot.theta))
            )

            window_surface.blit(sensor_distance, sensor_rectangle)

        pygame.display.update()
