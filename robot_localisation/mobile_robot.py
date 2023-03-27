import numpy as np

robot_radius = 52
robot_border_size = 2

"""
The following values are for the values of noise
"""

##### MOTION MODEL ERRORS #####
r_x, r_y, r_theta = 1, 1, 0.1

##### CONTROL ERRORS #####
e_vel, e_omega = 0.5, 0.03

##### ROBOT POSE ERRORS ####
pose_x, pose_y, pose_theta = 0.01, 0.01, 0.01

#### ROBOT SENSOR ERRORS ####
sensor_x, sensor_y, sensor_theta = 1, 1, 0.01

# Covariance matrix for motion model errors
R = np.matrix([
    [r_x ** 2, 0, 0],
    [0, r_y ** 2, 0],
    [0, 0, r_theta ** 2]
])

# Covariance matrix for sensor errors
Q = np.matrix([
    [sensor_x ** 2, 0, 0],
    [0, sensor_y ** 2, 0],
    [0, 0, sensor_theta ** 2]
])

# How the state evolves from t-1 -> t, without controls/noise
A = np.identity(3)

# How to map state to observation
C = np.identity(3)

horizontal_vector = [1, 0]


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


def feature_inclination(feature_vector: list):

    feature_points_up = feature_vector[1] > 0

    unit_vector_1 = horizontal_vector / np.linalg.norm(horizontal_vector)
    unit_vector_2 = feature_vector / np.linalg.norm(feature_vector)
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    angle = np.arccos(dot_product) if feature_points_up else -np.arccos(dot_product)

    return angle


def bearing_to_feature(heading_vector: list, feature_vector: list):

    feature_points_up = feature_vector[1] > 0

    unit_vector_1 = heading_vector / np.linalg.norm(heading_vector)
    unit_vector_2 = feature_vector / np.linalg.norm(feature_vector)
    dot_product = np.dot(unit_vector_1, unit_vector_2)

    angle = np.arccos(dot_product) if feature_points_up else -np.arccos(dot_product)

    return angle

def bearing_to_feature_atan2(landmark, robot_pos):
    return np.arctan2(landmark[1]-robot_pos[1], landmark[0] - robot_pos[0]) - robot_pos[2]

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

        # Initial state is known pose
        self.state = np.matrix([
            [self.pos[0]],
            [self.pos[1]],
            [self.theta]
        ])

        # Covariance matrix for pose error
        self.sigma = np.matrix([
            [pose_x ** 2, 0, 0],
            [0, pose_y ** 2, 0],
            [0, 0, pose_theta ** 2]
        ])

    def update_position(self, dt=1):
        """
        Updates the robot's position over time, dt

        :param dt: Integer indicating the size of the time step
        :return: None
        """

        x, y, theta = self.calculate_position(dt)
        corrected_pos, corrected_cov, did_correction_step = self.kalman_filter(dt)

        self.pos = x, y
        self.theta = theta

        self.state = corrected_pos
        self.sigma = corrected_cov

        pos = (x, y)
        corr_pos = (corrected_pos.item(0, 0), corrected_pos.item(1, 0))
        corr_cov = (corrected_cov.item(0, 0), corrected_cov.item(1, 1))

        '''print(f"The new position of the robot: {self.pos}")
        print(f"The new state of the robot: {corr_pos}")
        print(f"The predicted state of the robot: {pred_pos}")
        print(f"The predicted covariance:\n{predicted_cov}")
        print(f"The new covariance:\n{corrected_cov}\n---------------")'''

        return pos, corr_pos, corr_cov, did_correction_step

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

        # Control
        velocity = self.velocity
        omega = self.omega

        error_vel, error_omega = np.random.normal(0, scale=e_vel),\
                                np.random.normal(0, scale=e_omega)


        control = np.matrix([
            [self.velocity + error_vel],
            [self.omega + error_omega]
        ])

        # Matrix describing how control changes pose from t-1 -> t
        B = np.matrix([
            [dt * np.cos(self.state.item(2, 0)), 0],
            [dt * np.sin(self.state.item(2, 0)), 0],
            [0, dt]
        ])

        predicted_position = (A * self.state) + (B * control)

        predicted_sigma = (A * self.sigma * np.transpose(A)) + R
        #print(f"\nThe current state is: \n{self.state}")

        #### CORRECTION ####
        x, y = self.pos[0], self.pos[1]

        count_features_in_range = 0
        features_in_range = []
        for feature in self.features:
            distance = distance_to_feature((x, y), feature)
            if distance < self.sensor_range + robot_radius:
                 count_features_in_range += 1
                 features_in_range.append(feature)
        
        did_correction_step = False
        if count_features_in_range >= 3:
            observation = self.get_observation2(features_in_range)
            #print(f"\nThe observation is: \n{observation}")

            C_T = np.transpose(C)
            kalman_gain = predicted_sigma * C_T * np.linalg.inv((C * predicted_sigma * C_T) + Q)
            corrected_position = predicted_position + (kalman_gain * (observation - (C * predicted_position)))

            #print(f"\nCorrection position by KF:\n{corrected_position}")
            did_correction_step = True
            corrected_sigma = (np.identity(3) - (kalman_gain * C)) * predicted_sigma
        else:
            corrected_position = predicted_position
            corrected_sigma = predicted_sigma

        return corrected_position, corrected_sigma, did_correction_step

    def get_observation2(self, features_in_range):
        x, y = self.pos[0], self.pos[1]
        theta = self.theta



        theta_estimates = []
        for feature in features_in_range:
            feature_vector = [feature[0] - x, feature[1] - y]

            alpha = bearing_to_feature_atan2([feature[0], feature[1]], [x,y,theta])
            beta = feature_inclination(feature_vector)
            theta_estimates.append(beta - alpha)
        
        theta_avg = np.mean(theta_estimates)
        error_x, error_y, error_theta = np.random.normal(0, scale=sensor_x),\
                                        np.random.normal(0, scale=sensor_y), \
                                        np.random.normal(0, scale=sensor_theta)
        
        observation = np.matrix([
            [x + error_x],
            [y + error_y],
            [theta_avg + error_theta]
        ])
        return observation

    def get_observation(self):
        """
        Get the pose based on observed features, using triangulation

        :return: Returns a np matrix containing average, x y and theta
        """

        # state_x, state_y, state_theta = self.state.item(0, 0), self.state.item(1, 0), self.state.item(2, 0)
        state_x, state_y, state_theta = self.pos[0], self.pos[1], self.theta
        #print("Theta is: ", str(self.theta))

        heading_vector = [robot_radius * np.cos(state_theta), robot_radius * np.sin(state_theta)]

        # These hold the estimated pose for each feature
        x, y, theta = [], [], []

        for feature in self.features:

            distance = distance_to_feature((state_x, state_y), feature)

            if distance < robot_radius + self.sensor_range:

                feature_vector = [feature[0] - state_x, feature[1] - state_y]

                alpha = bearing_to_feature(heading_vector, feature_vector)
                beta = feature_inclination(feature_vector)

                corrected_x = feature[0] - distance * np.cos(alpha)
                corrected_y = feature[1] - distance * np.sin(alpha)
                corrected_theta = beta - alpha

                #print(f"Feature {feature} suggests the robot is at:\n"
                #      f"{corrected_x}, {corrected_y} with angle: {np.degrees(corrected_theta)}")

                x.append(corrected_x)
                y.append(corrected_y)
                theta.append(corrected_theta)

        x_avg, y_avg, theta_avg = np.mean(x), np.mean(y), np.mean(theta)
        error_x, error_y, error_theta = np.random.normal(0, scale=sensor_x),\
                                        np.random.normal(0, scale=sensor_y), \
                                        np.random.normal(0, scale=sensor_theta)

        #print(f"The avg pose: {x_avg, y_avg, np.degrees(theta_avg)}")

        #print(f"Errors on pose: {error_x, error_y, np.degrees(error_theta)}")

        observation = np.matrix([
            [x_avg + error_x],
            [y_avg + error_y],
            [theta_avg + error_theta]
        ])

        return observation
