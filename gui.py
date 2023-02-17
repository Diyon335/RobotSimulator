import pygame
from sympy import Point, Line

is_running = True
window_size = (900, 800)

change_robot_size = False
new_robot_size = (50, 50)

walls = [
    Line(Point(110, 90), Point(810, 90)),
    Line(Point(110, 90), Point(110, 710)),
    Line(Point(110, 710), Point(810, 710)),
    Line(Point(810, 710), Point(810, 90))
]


def run(robot):

    pygame.init()

    # Create display
    pygame.display.set_caption("Robot Simulator")
    window_surface = pygame.display.set_mode(window_size)

    # Create background
    background = pygame.Surface(window_size)
    background.fill(pygame.Color('#FFFFFF'))

    robot_image = pygame.image.load("robot.png")

    # Default size for the robot is 100x100 pixels. If this needs to be changed, set change_robot_size = True and
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
            if event.type == pygame.MOUSEBUTTONDOWN:
                robot_image = pygame.transform.rotate(robot_image, robot.theta - 90)

        # Draw background, robot and walls
        window_surface.blit(background, (0, 0))
        window_surface.blit(robot_image, robot.pos)

        for wall in walls:
            pygame.draw.line(window_surface, "#000000", wall.p1, wall.p2, width=2)

        pygame.display.update()
