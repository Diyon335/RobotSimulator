import math

from sympy import Point, Line, Segment, Ellipse
from gui import robot_radius, robot_border_size
import numpy as np


class Robot:
    """
    Differential Drive Robot Object
    Contains positional information about the robot as well as
    """

    class Sensor:
        """
        Nested class for a sensor object
        """

        def __init__(self, sensor_range, angle, robot):
            """
            Constructor for a sensor

            :param sensor_range: Maximum detection range of the sensor
            :param angle: Angle of placement on the robot's edge. The angle is with respect to the x-axis
            :param robot: Robot object
            """

            self.sensor_range = sensor_range
            self.angle = angle
            self.robot = robot

            # Starting point of sensor
            self.p1 = None
            # Point until which it can see
            self.p2 = None

        def sense_distance(self):
            """
            Computes the distance from the edge of the robot to the nearest wall
            """

            # The starting point is the robot's centre
            starting_point = Point(self.robot.pos[0], self.robot.pos[1])

            # Ending point is the sensor's max distance away from from its starting point, in the direction of the
            # sensor's angle plus the robot's radius
            ending_x = starting_point[0] \
                + self.sensor_range * math.cos(math.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * math.cos(math.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * math.cos(math.radians(self.robot.theta - self.angle - 90))

            ending_y = starting_point[1] \
                + self.sensor_range * math.sin(math.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * math.sin(math.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * math.sin(math.radians(self.robot.theta - self.angle - 90))

            ending_point = Point(ending_x, ending_y)

            # To help plotting the sensor line on the GUI if needed
            self.p1 = starting_point
            self.p2 = ending_point

            # Changed Line -> Segment. Line results in errors when joining two points like this
            sensor_detection_line = Segment(starting_point, ending_point)

            distances_list = []

            for line in self.robot.room_map:

                # Returns a list with possible intersections
                intersection_points = sensor_detection_line.intersection(line)

                # If no intersections, continue
                if len(intersection_points) < 1:
                    continue

                # Add distance of each intersection
                for intersection in intersection_points:
                    distances_list.append(starting_point.distance(intersection))

            # Return the sensor's max range if there are no intersections, else return the min distance
            return self.sensor_range if len(distances_list) < 1 else min(distances_list) - robot_radius + robot_border_size

    def __init__(self, robot_id, pos, room_map, v_r=0, v_l=0, theta=90, n_sensors=12):
        self.robot_id = robot_id
        self.pos = pos
        self.v_l = v_l
        self.v_r = v_r
        self.theta = theta
        self.room_map = room_map
        self.sensors = self.generate_sensors(n_sensors)

    def set_pos(self, x, y):
        if x is None or y is None:
            raise TypeError("Unsupported operand")

        self.pos = (x, y)

    def set_vel_left(self, v):
        if v is None:
            raise TypeError("Unsupported operand", type(v))

        self.v_l = v

    def set_vel_right(self, v):
        if v is None:
            raise TypeError("Unsupported operand", type(v))

        self.v_r = v

    def set_angle(self, theta):
        if theta is None:
            raise TypeError("Unsupported operand", type(theta))

        self.theta = theta

    def generate_sensors(self, n_sensors):
        return [self.Sensor(200, angle, self) for angle in range(0, 360, int(360 / n_sensors))]

    def update_position(self):
        # Function to update the Robot position based on current position,
        # wheel velocities and environment

        # Remember that in the edge case: we have to recursively update position

        # Get new prospective position
        new_pos_orient = self.get_new_pos()
        new_theta = new_pos_orient[2]
        new_pos = (new_pos_orient[0], new_pos_orient[1])

        # Recursive check for legal position
        legal = False
        while not legal:
            new_pos, legal = self.correct_pos(new_pos)

        # Set new position and orientation
        self.pos = new_pos
        self.theta = new_theta

        # Set new position

    def get_new_pos(self):
        # 1: Calculate omega and R
        l = robot_radius
        
        if self.v_l == self.v_r:
            return self.pos[0] + self.v_l * np.cos(np.radians(self.theta)), \
                   self.pos[1] + self.v_l * np.sin(np.radians(self.theta)), self.theta

        R = l * ((self.v_l + self.v_r) / (self.v_r - self.v_l))
        omega = (self.v_r - self.v_l) / (l*2)
        x = self.pos[0]
        y = self.pos[1]

        # 2: Calculate coordinates for ICC
        ICC_x = x - R * np.sin(np.radians(self.theta))
        ICC_y = y + R * np.cos(np.radians(self.theta))

        # 3: Calculate new position and orientation of robot and return
        x_new = (np.cos(np.radians(omega)) * (x - ICC_x) - np.sin(np.radians(omega)) * (y - ICC_y)) + ICC_x
        y_new = (np.sin(np.radians(omega)) * (x - ICC_x) + np.cos(np.radians(omega)) * (y - ICC_y)) + ICC_y
        theta_new = self.theta - omega

        return x_new, y_new, theta_new

    def correct_pos(self, new_pos):
        # Checks if the new position of the robot is legal and returns a position
        # that is more legal (guaranteed improvement but not guaranteed to be legal)

        # Create sympy circle representing robot at new location, 
        # and sympy line between old and new position
        old_center = Point(self.pos[0], self.pos[1])
        new_center = Point(new_pos[0], new_pos[1])
        robot_circle = Ellipse(center=new_center, hradius=robot_radius, vradius=robot_radius)
        travel_path = Segment(old_center, new_center)
        
        # Check for intersections with map (both circle and line) store the intersection point
        # and which line those intersections are associated with
        distances_dict = {}
        
        for line in self.room_map:
            intersections_circle = robot_circle.intersection(line)
            intersections_path = travel_path.intersection(line)
            
            if len(intersections_path) < 1 and len(intersections_circle) < 1:
                continue

            distances = []
            for intersect in intersections_circle:
                distances.append(old_center.distance(intersect))
            
            if len(intersections_path) > 0:
                distances.append(old_center.distance(intersections_path[0]))
            distances_dict[line] = min(distances)
        # If there are are no intersections: return new_pos, True
        if not distances_dict:
            return new_pos, True

        # Check which intersection with the map occurs closest to the robot's current position
        closest_line = min(distances_dict, key=distances_dict.get)
        # If the line is vertical (x1 = x2) shift the robots new x coordinate so that it lies one
        # radius away from the line (in the direction of the robot)

        # If the line is horizontal (y1 = y2) Shift the robots new y coordinate so that it lies one
        # radius away from the line
        p1 = closest_line.p1
        p2 = closest_line.p2

        new_x = new_pos[0]
        new_y = new_pos[1]
        if p1[0] == p2[0]:
            if self.pos[0] < p1[0]:
                new_x = p1[0] - (robot_radius + 1)
            else:
                new_x = p1[0] + (robot_radius + 1)
        elif p1[1] == p2[1]:
            if self.pos[1] < p1[1]:
                new_y = p1[1] - (robot_radius + 1)
            else:
                new_y = p1[1] + (robot_radius + 1)

        return (new_x, new_y), False
