import math

from sympy import Point, Line
from gui import robot_size


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

            self.p1 = None
            self.p2 = None

        def sense_distance(self):
            """
            Computes the distance from the edge of the robot to the nearest wall
            """

            radius = robot_size[0]/2

            # The starting point is a radius's distance away from the centre of the robot, in the direction of the
            # sensor's angle
            starting_point = Point(self.robot.pos[0] + radius * math.cos(math.radians(self.angle)),
                                   self.robot.pos[1] + radius * math.sin(math.radians(self.angle)))

            # Ending point is the sensor's max distance away from from its starting point, in the direction of the
            # sensor's angle
            ending_x = starting_point.x + self.sensor_range * math.cos(math.radians(self.angle))
            ending_y = starting_point.y + self.sensor_range * math.sin(math.radians(self.angle))

            ending_point = Point(ending_x, ending_y)

            # Just to keep track of whether what we're doing is right
            self.p1 = starting_point
            self.p2 = ending_point

            sensor_detection_line = Line(starting_point, ending_point)

            distances_list = []

            for line in self.robot.room_map:
                intersection_point_list = sensor_detection_line.intersection(line)

                # If no intersection, then return its max range
                if len(intersection_point_list) < 1:
                    distances_list.append(self.sensor_range)
                    continue

                intersection_point = min(intersection_point_list)

                if intersection_point is not None:
                    distances_list.append(starting_point.distance(intersection_point))

            # print("All collisions with a particular line ", intersection_point_list)
            #
            # print("Minimum collision point with a particular line: ",intersection_point)
            #
            # print("List of all minimum collision points ", distances_list)
            #
            # print("Minimum collision point ", min(distances_list))

            return min(distances_list)

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
