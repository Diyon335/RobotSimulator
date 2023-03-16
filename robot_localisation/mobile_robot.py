import numpy as np

robot_radius = 52
robot_border_size = 2

"""
The following values are for the values of noise
"""

##### MOTION MODEL ERRORS #####
r_x, r_y, r_theta = 1, 1, 0.1

##### ROBOT POSE ERRORS ####
pose_x, pose_y, pose_theta = 0.01, 0.01, 0.01

# Covariance matrix for motion model errors
R = np.matrix([
    [r_x ** 2, 0, 0],
    [0, r_y ** 2, 0],
    [0, 0, r_theta ** 2]
])

# How the state evolves from t-1 -> t, without controls/noise
A = np.identity(3)

# How to map state to observation
C = np.identity(3)


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


class Robot:

    def __init__(self, position, sensor_range, room, theta=0, omega=0):
        """
        Constructor for the robot

        :param position: Tuple indicating the starting position
        :param sensor_range: Integer indicating the sensor range of the robot
        :param room: A list containing the walls and features of the room
        :param theta: Angle in radians for the robot's orientation
        :param omega: Angle in radians for the rate of change of the robot's orientation
        """

        self.pos = position
        self.velocity = 0
        self.theta = theta
        self.omega = omega
        self.sensor_range = sensor_range

        self.features = room[1]

        # Covariance matrix for pose error
        self.sigma = np.matrix([
            [pose_x ** 2, 0, 0],
            [0, pose_y ** 2, 0],
            [0, 0, pose_theta ** 2]
        ])

    def update_position(self, dt=1, kf=False):
        """
        Updates the robot's position over time, dt

        :param kf: If true, return predicted and corrected positions, along with the covariance matrix sigma
        :param dt: Integer indicating the size of the time step
        :return: None
        """

        x, y, theta = self.calculate_position(dt)

        if kf:

            predicted_pos, corrected_pos, covariance = self.kalman_filter(dt)

            self.pos = x, y
            self.theta = theta

            return predicted_pos, corrected_pos, covariance

        self.pos = x, y
        self.theta = theta

    def calculate_position(self, dt):

        # Current pose
        current_position = np.matrix([
            [self.pos[0]],
            [self.pos[1]],
            [self.theta]
        ])

        # Control
        control = np.matrix([
            [self.velocity],
            [self.omega]
        ])

        # Matrix describing how control changes pose from t-1 -> t
        B = np.matrix([
            [dt * np.cos(self.theta), 0],
            [dt * np.sin(self.theta), 0],
            [0, dt]
        ])

        new_position = current_position + (B * control)

        return new_position.item(0, 0), new_position.item(1, 0), new_position.item(2, 0)

    def kalman_filter(self, dt):
        """
        Uses the velocity based model to compute the robot's new position

        :param dt: Integer for the time step
        :return: Returns a tuple with the robot's new x, y and theta
        """

        #### PREDICTION ####

        # Current pose
        current_position = np.matrix([
            [self.pos[0]],
            [self.pos[1]],
            [self.theta]
        ])

        # Control
        u = np.matrix([
            [self.velocity],
            [self.omega]
        ])

        # Matrix describing how control changes pose from t-1 -> t
        B = np.matrix([
            [dt * np.cos(self.theta), 0],
            [dt * np.sin(self.theta), 0],
            [0, dt]
        ])

        predicted_position = (A * current_position) + (B * u)

        predicted_sigma = (A * self.sigma * np.transpose(A)) + R

        #### CORRECTION ####








