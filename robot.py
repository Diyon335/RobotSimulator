import math

from sympy import Point, Line

"""
   Differential Drive Robot Object
   Contains positional information about the robot as well as  
"""
class Robot:

    class Sensor:

        def __init__(self, sensor_range, angle, robot):

            self.sensor_range = sensor_range
            self.angle = angle
            self.robot = robot

        def sense_distance(self):

            """TODO: subract the radius to ending_x and ending_y parameters"""

            starting_point = Point(self.robot.pos[0], self.robot.pos[1])
            ending_x = self.sensor_range * math.cos(math.radians(self.angle))
            ending_y = self.robot.pos[1] + self.sensor_range * math.sin(math.radians(self.angle))
            ending_point = Point(ending_x, ending_y)

            sensor_detection_line = Line(starting_point, ending_point)

            distances_list = []
            for line in self.robot.room_map:
                intersection_point_list = sensor_detection_line.intersection(line)
                intersection_point = min(intersection_point_list)
                if intersection_point != None:
                    distances_list.append(starting_point.distance(intersection_point))

            print("All collisions with a particular line ", intersection_point_list)

            print("Minimum collision point with a particular line: ",intersection_point)

            print("List of all minimum collision points ", distances_list)

            print("Minimum collision point ", min(distances_list))

            return min(distances_list)
            """
            return sensor_range if (wall_line - (robot_centre + robot_radius)) > sensor_range \
                    else (wall_line - (robot_centre + robot_radius))
            """

            """
            Get sympy line for the wall
            Create sympy line going from the sensor
            Check intersection between the lines
            Calculate distances from sensor to wall
            Return the min distance
            """

    def __init__(self, robot_id, pos, room_map, v_r=0, v_l=0, theta=0, n_sensors=12):
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

        sensors_list = []
        sensor_1 = self.Sensor(10, 30, self)
        sensor_2 = self.Sensor(10, 60, self)
        sensors_list.append(sensor_1)
        sensors_list.append(sensor_2)
        return sensors_list

    def update_position(self):
        pass
        # TODO: Implement function to update the Robot position based on current position,
        # wheel velocities and environment

        # Remember that in the edge case: we have to recursively update position
