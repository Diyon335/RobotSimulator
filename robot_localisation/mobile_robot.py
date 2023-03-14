import numpy as np

robot_radius = 52
robot_border_size = 2


class Robot:

    def __init__(self, position, sensor_range, theta=np.pi/2, omega=0):
        """
        Constructor for the robot

        :param position: Tuple indicating the starting position
        :param sensor_range: Integer indicating the sensor range of the robot
        :param theta: Angle in radians for the robot's orientation
        :param omega: Angle in radians for the rate of change of the robot's orientation
        """

        self.pos = position
        self.velocity = 0
        self.theta = theta
        self.omega = omega
        self.sensor_range = sensor_range

    def update_position(self, dt=1):
        """
        Updates the robot's position over time, dt

        :param dt: Integer indicating the size of the time step
        :return: None
        """

        x, y, theta = self.calculate_position(dt)
        self.pos = x, y
        self.theta = theta

    def calculate_position(self, dt):
        """
        Uses the velocity based model to compute the robot's new position

        :param dt: Integer for the time step
        :return: Returns a tuple with the robot's new x, y and theta
        """

        current_position = np.matrix([
            [self.pos[0]],
            [self.pos[1]],
            [self.theta]
        ])

        B = np.matrix([
            [dt * np.cos(self.theta), 0],
            [dt * np.sin(self.theta), 0],
            [0, dt]
        ])

        u = np.matrix([
            [self.velocity],
            [self.omega]
        ])

        new_position = current_position + (B * u)

        return new_position.item(0, 0), new_position.item(1, 0), new_position.item(2, 0)


