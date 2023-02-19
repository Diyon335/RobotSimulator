import math

from sympy import Point, Line, Segment
from gui import robot_size, robot_border_size


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

            radius = robot_size[0]/2

            # The starting point is a radius's distance away from the centre of the robot minus the border of the
            # robot's outline, in the direction of the sensor's angle
            starting_point = Point(self.robot.pos[0] + radius * math.cos(math.radians(self.angle))
                                   - robot_border_size * math.cos(math.radians(self.angle)),

                                   self.robot.pos[1] + radius * math.sin(math.radians(self.angle))
                                   - robot_border_size * math.sin(math.radians(self.angle))
                                   )

            # Ending point is the sensor's max distance away from from its starting point, in the direction of the
            # sensor's angle
            ending_x = starting_point.x + self.sensor_range * math.cos(math.radians(self.angle))
            ending_y = starting_point.y + self.sensor_range * math.sin(math.radians(self.angle))

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
            return self.sensor_range if len(distances_list) < 1 else min(distances_list)

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
        return [self.Sensor(200, angle, self) for angle in range(0, 360, int(360/n_sensors))]

    def update_position(self):
        pass
        # TODO: Implement function to update the Robot position based on current position,
        # wheel velocities and environment

        # Remember that in the edge case: we have to recursively update position
