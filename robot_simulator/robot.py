"""
Autonomous Robotic Systems (2223-KEN4114)
Assignment 2: Robot Simulator

@authors:
Diyon Wickrameratne (i6176139)
Luca Forte (I6330944)
Olmo Denegri (i6333396)
Florent Didascalou (i6337071)
"""

from sympy import Point, Segment, Ellipse
from robot_simulator.gui import robot_radius, robot_border_size
import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point as SPoint
from shapely.geometry import MultiPoint

horizontal_vector = [1, 0]


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
                + self.sensor_range * np.cos(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * np.cos(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * np.cos(np.radians(self.robot.theta - self.angle - 90))

            ending_y = starting_point[1] \
                + self.sensor_range * np.sin(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * np.sin(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * np.sin(np.radians(self.robot.theta - self.angle - 90))

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

        def sense_distance2(self):
            """
            Computes the distance from the edge of the robot to the nearest wall
            """

            # The starting point is the robot's centre
            starting_point = SPoint(self.robot.pos)

            # Ending point is the sensor's max distance away from from its starting point, in the direction of the
            # sensor's angle plus the robot's radius
            ending_x = starting_point.x \
                + self.sensor_range * np.cos(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * np.cos(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * np.cos(np.radians(self.robot.theta - self.angle - 90))

            ending_y = starting_point.y \
                + self.sensor_range * np.sin(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_radius * np.sin(np.radians(self.robot.theta - self.angle - 90)) \
                + robot_border_size * np.sin(np.radians(self.robot.theta - self.angle - 90))

            ending_point = SPoint((ending_x, ending_y))

            # To help plotting the sensor line on the GUI if needed
            self.p1 = starting_point
            self.p2 = ending_point

            # Changed Line -> Segment. Line results in errors when joining two points like this
            sensor_detection_line = LineString([starting_point, ending_point])

            distances_list = []

            for line in self.robot.room_map:

                # Returns a list with possible intersections
                intersection_points = sensor_detection_line.intersection(line)

                # If no intersections, continue
                if intersection_points.is_empty:
                    continue

                if type(intersection_points) == SPoint:
                    distances_list.append(starting_point.distance(intersection_points))

                if type(intersection_points) == MultiPoint:
                    for i in range(len(intersection_points.geoms)):
                        distances_list.append(starting_point.distance(intersection_points.geoms[i]))

            # Return the sensor's max range if there are no intersections, else return the min distance
            return self.sensor_range if len(distances_list) < 1 else min(distances_list) - robot_radius + robot_border_size - 2

    def __init__(self, robot_id, pos, room_map, v_r=0, v_l=0, theta=90, n_sensors=12):
        self.robot_id = robot_id
        self.pos = pos
        self.v_l = v_l
        self.v_r = v_r
        self.theta = theta
        self.room_map = room_map[0]
        self.sensors = self.generate_sensors(n_sensors)
        self.dust = 0

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
        """
        Updates the position of the robot. It applies a recursive check to correct the robot's position
        :return:
        """
        old_pos = self.pos

        # Get new prospective position
        new_pos_orient = self.get_new_pos()
        new_theta = new_pos_orient[2]
        new_pos = (new_pos_orient[0], new_pos_orient[1])

        i = 0
        legal = False
        while not legal:
            new_pos, legal = self.correct_pos2(old_pos, new_pos)

            if i > 5:
                break

            i += 1

        # Set new position and orientation
        self.pos = new_pos
        self.theta = new_theta

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
        x_new = (np.cos(omega) * (x - ICC_x) - np.sin(omega) * (y - ICC_y)) + ICC_x
        y_new = (np.sin(omega) * (x - ICC_x) + np.cos(omega) * (y - ICC_y)) + ICC_y
        theta_new = self.theta - np.degrees(omega)

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
        min_distance = min(distances_dict.values())
        closest_lines = [key for key in distances_dict if distances_dict[key] == min_distance]
        
        # Further check in the case of intersecting at a corner to prevent illegal moves
        closest_line = closest_lines[0]
        if len(closest_lines) > 1:
            for i in range(1, len(closest_lines)):
                if closest_lines[i].distance(old_center) < closest_line.distance(old_center):
                    closest_line = closest_lines[i]

        # closest_line = min(distances_dict, key=distances_dict.get)

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
    
    def correct_pos_shapely(self, new_pos):
        old_center = SPoint(self.pos)
        new_center = SPoint(new_pos)
        robot_circle = new_center.buffer(robot_radius).boundary

        travel_path = LineString([self.pos, new_pos])
        
        distances_dict = {}
        for wall in self.room_map:
            circle_intersections = robot_circle.intersection(wall)
            line_intersections = travel_path.intersection(wall)
            if circle_intersections.is_empty and line_intersections.is_empty:
                continue
            
            distances = []
            if type(circle_intersections) is MultiPoint:
                
                for p in circle_intersections.geoms:
                    distances.append(old_center.distance(p))
            elif type(circle_intersections) is SPoint:
                distances.append(old_center.distance(circle_intersections))
            
            if type(line_intersections) is SPoint:
                distances.append(old_center.distance(line_intersections))
            elif type(line_intersections) is MultiPoint:
                distances.append(old_center.distance(wall))
            distances_dict[wall] = min(distances)
        
        if not distances_dict:
            return new_pos, True

        # Check which intersection with the map occurs closest to the robot's current position
        min_distance = min(distances_dict.values())
        closest_lines = [key for key in distances_dict if distances_dict[key] == min_distance]

        # Further check in the case of intersecting at a corner to prevent illegal moves
        closest_line = closest_lines[0]
        if len(closest_lines) > 1:
            for i in range(1, len(closest_lines)):
                if closest_lines[i].distance(old_center) < closest_line.distance(old_center):
                    closest_line = closest_lines[i]

        p1 = (list(closest_line.coords)[0][0],  list(closest_line.coords)[0][1])
        p2 = (list(closest_line.coords)[1][0],  list(closest_line.coords)[1][1])

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

    def correct_pos2(self, old_pos, new_pos):
        """
        A robot's new position is correct if there is a collision between walls of the room

        If there is no collision, the new position is just returned

        There are three types of collisions:

        1) Circle to wall collisions: The circle colliding with a wall is resolved by positioning the robot's centre a
        radius' distance away from the point of intersection along the normal to the wall. If the circle collides with
        an outward pointing corner, the robot is placed a radius' distance away from the intersection point of the two
        walls in the direction of the shortest line connecting the intersection point and the robot's centre

        2) Line (connecting the two centres) to wall collision: If the speed of the robot is too high, then it will pass
        through a wall entirely. Therefore, a line is drawn from the robot's centre from its old position to its new
        position. The point where this line intersects the closest wall is the intersection point. The robot is then
        placed a radius' distance away from the intersection point along the normal line to the wall

        3) Border lines to wall collision: These lines are those that connect the robot's right and left most points
        on the circle. The same method of resolution as described in (2) is applied here

        :param old_pos: Tuple of the robot's old position
        :param new_pos: Tuple of the robot's new position
        :return: A tuple containing the (resolved) position of the robot and a Boolean indicating if its (resolved)
        position is legal
        """

        x, y = old_pos
        lx, ly = x + robot_radius * np.cos(np.radians(self.theta - 90)), \
                 y + robot_radius * np.sin(np.radians(self.theta - 90))

        rx, ry = x + robot_radius * np.cos(np.radians(self.theta + 90)), \
                 y + robot_radius * np.sin(np.radians(self.theta + 90))

        x_new, y_new = new_pos
        lx_new, ly_new = x_new + robot_radius * np.cos(np.radians(self.theta - 90)), \
                         y_new + robot_radius * np.sin(np.radians(self.theta - 90))

        rx_new, ry_new = x_new + robot_radius * np.cos(np.radians(self.theta + 90)), \
                         y_new + robot_radius * np.sin(np.radians(self.theta + 90))

        old_centre = SPoint(old_pos)
        new_centre = SPoint(new_pos)
        robot_circle = new_centre.buffer(robot_radius).boundary

        old_left = SPoint((lx, ly))
        old_right = SPoint((rx, ry))

        new_left = SPoint((lx_new, ly_new))
        new_right = SPoint((rx_new, ry_new))

        left_line = LineString([old_left, new_left])
        right_line = LineString([old_right, new_right])
        travelled_path = LineString([old_pos, new_pos])

        circle_distances = {}
        line_distances = {}
        left_right_line_distances = {}

        for wall in self.room_map:
            circle_intersection_points = robot_circle.intersection(wall)
            line_intersection_points = travelled_path.intersection(wall)

            left_line_intersection_points = left_line.intersection(wall)
            right_line_intersection_points = right_line.intersection(wall)

            # Do a preliminary check to see if there are any intersections first, to avoid useless calculations
            if circle_intersection_points.is_empty and line_intersection_points.is_empty \
                    and left_line_intersection_points.is_empty and right_line_intersection_points.is_empty:

                continue

            if not circle_intersection_points.is_empty:
                # print("this happened")
                # Does the circle intersect at two points on the same wall? --> midpoint
                if type(circle_intersection_points) is MultiPoint:

                    if len(circle_intersection_points.geoms) == 2:
                        p1 = SPoint((circle_intersection_points.geoms[0].x, circle_intersection_points.geoms[0].y))
                        p2 = SPoint((circle_intersection_points.geoms[1].x, circle_intersection_points.geoms[1].y))

                        mid_x = (p1.x + p2.x)/2
                        mid_y = (p1.y + p2.y)/2

                        mid_point = SPoint((mid_x, mid_y))
                        distance_to_point = old_centre.distance(mid_point)
                        circle_distances[(distance_to_point, wall)] = mid_point

                        # This info is enough to correct the position
                        continue

                # Does the circle intersect just one point on the wall? Add it as a single point
                if type(circle_intersection_points) == SPoint:

                    distance_to_point = old_centre.distance(circle_intersection_points)
                    circle_distances[(distance_to_point, wall)] = circle_intersection_points

                    # This info is enough to correct the position
                    continue

            # If no circle intersections, there could be line intersections
            if not line_intersection_points.is_empty:

                # If a single point
                if type(line_intersection_points) == SPoint:

                    distance_to_point = old_centre.distance(line_intersection_points)
                    line_distances[(distance_to_point, wall)] = line_intersection_points

                # If multiple points - very rare case
                if type(line_intersection_points) == MultiPoint:

                    for i in range(len(line_intersection_points.geoms)):

                        distance_to_point = old_centre.distance(line_intersection_points.geoms[i])
                        line_distances[(distance_to_point, wall)] = line_intersection_points.geoms[i]

            if not left_line_intersection_points.is_empty:

                # If a single point
                if type(left_line_intersection_points) == SPoint:
                    distance_to_point = old_centre.distance(left_line_intersection_points)
                    left_right_line_distances[(distance_to_point, wall)] = left_line_intersection_points

                # If multiple points - very rare case
                if type(left_line_intersection_points) == MultiPoint:

                    for i in range(len(left_line_intersection_points.geoms)):
                        distance_to_point = old_centre.distance(left_line_intersection_points.geoms[i])
                        left_right_line_distances[(distance_to_point, wall)] = left_line_intersection_points.geoms[i]

            if not right_line_intersection_points.is_empty:

                # If a single point
                if type(right_line_intersection_points) == SPoint:
                    distance_to_point = old_centre.distance(right_line_intersection_points)
                    left_right_line_distances[(distance_to_point, wall)] = right_line_intersection_points

                # If multiple points - very rare case
                if type(right_line_intersection_points) == MultiPoint:

                    for i in range(len(right_line_intersection_points.geoms)):
                        distance_to_point = old_centre.distance(right_line_intersection_points.geoms[i])
                        left_right_line_distances[(distance_to_point, wall)] = right_line_intersection_points.geoms[i]

        # If no intersections with all walls, just return new pos
        if len(circle_distances) < 1 and len(line_distances) < 1 and len(left_right_line_distances) < 1:
            return new_pos, True

        # Circle intersections
        if len(circle_distances) > 0:

            minimum_distance, wall = min(circle_distances.keys(), key=lambda t: t[0])
            closest_point = circle_distances[minimum_distance, wall]

            if len(circle_distances) == 2:
                walls = [key[1] for key in circle_distances.keys()]
                wall1, wall2 = walls[0], walls[1]

                wall_intersection = wall1.intersection(wall2)

                if not wall_intersection.is_empty and not self.is_orthogonal(wall1, wall2) and \
                        (new_centre.distance(wall_intersection) <= minimum_distance):

                    short_line_inclination = self.get_shortest_line_inclination(wall_intersection, new_pos)

                    return self.get_corrected_xy(wall_intersection, short_line_inclination, corner=True), False

            normal_line = self.get_normal_vector(wall)
            normal_line_inclination = self.get_normal_line_inclination(normal_line)

            return self.get_corrected_xy(closest_point, normal_line_inclination), False

        # Border intersections
        if len(left_right_line_distances) > 0:

            minimum_distance, wall = min(left_right_line_distances.keys(), key=lambda t: t[0])
            closest_point = left_right_line_distances[minimum_distance, wall]

            normal_line = self.get_normal_vector(wall)
            normal_line_inclination = self.get_normal_line_inclination(normal_line)

            return self.get_corrected_xy(closest_point, normal_line_inclination), False

        # Line intersections
        if len(line_distances) > 0:

            minimum_distance, wall = min(line_distances.keys(), key=lambda t: t[0])
            closest_point = line_distances[minimum_distance, wall]

            normal_line = self.get_normal_vector(wall)
            normal_line_inclination = self.get_normal_line_inclination(normal_line)

            return self.get_corrected_xy(closest_point, normal_line_inclination), False

    def get_corrected_xy(self, intersection_point, normal_line_inclination, corner=False):
        """
        Gets the corrected centre position of the robot

        :param corner: A boolean indicating whether the correction is being performed for a wall or outward corner
        :param normal_line_inclination: An angle in degrees
        :param intersection_point: SPoint object indicating the closest intersection point
        intersection point
        :return: A tuple of integers indicating the robot's new centre (x, y)
        """

        x, y = intersection_point.x, intersection_point.y

        # k is a constant to differentiate the offsets from a wall and corner offset
        k = 1 if not corner else -1

        # Here we calculate the offset of intersection point, so that the new centre lies a radius away from the
        # intersection point
        x_offset = k * (robot_radius + robot_border_size) * np.cos(normal_line_inclination)
        y_offset = -(robot_radius + robot_border_size) * np.sin(normal_line_inclination)

        new_x, new_y = x + x_offset, y + y_offset

        return new_x, new_y

    def get_normal_vector(self, wall):
        """
        Gets the normal vector to a wall

        :param wall: A shapely LineString object
        :return: Returns a normalised normal vector in the form of a list
        """
        x1, y1 = wall.coords[0][0], wall.coords[0][1]
        x2, y2 = wall.coords[1][0], wall.coords[1][1]

        normal_vector = [-(y1 - y2), x1 - x2]

        return normal_vector / np.linalg.norm(normal_vector)

    def get_normal_line_inclination(self, normal_vector):
        """
        Gets the inclination of a normal vector with respect to the horizontal x-increasing axis

        :param normal_vector: A list indicating the increments in the x and y direction
        :return: Returns an angle in degrees
        """
        # This is an indication whether this vector points up or down
        shortest_line_points_up = normal_vector[1] < 0

        # Normalise the vector
        unit_vector_1 = horizontal_vector / np.linalg.norm(horizontal_vector)

        dot_product = np.dot(unit_vector_1, normal_vector)
        angle = np.arccos(dot_product) if shortest_line_points_up else -np.arccos(dot_product)

        return angle

    def is_orthogonal(self, line_1, line_2):
        """
        Checks whether two lines are orthogonal

        :param line_1: LineString shapely object
        :param line_2: LineString shapely object
        :return: Returns true if two LineString objects are orthogonal, i.e. the angle between them is 90 degrees
        """

        x1, y1 = line_1.coords[0][0], line_1.coords[0][1]
        x2, y2 = line_1.coords[1][0], line_1.coords[1][1]

        x1_, y1_ = line_2.coords[0][0], line_2.coords[0][1]
        x2_, y2_ = line_2.coords[1][0], line_2.coords[1][1]

        vector_1 = [x1 - x2, y1 - y2]
        vector_2 = [x1_ - x2_, y1_ - y2_]
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)

        dot_product = np.dot(unit_vector_1, unit_vector_2)

        return np.abs(np.degrees(np.arccos(dot_product))) == 90

    def get_shortest_line_inclination(self, intersection_point, new_pos):
        """
        The shortest line is the one that connects the centre of the robot and an intersection point. This function
        returns its inclination with respect to the positive x-axis in the clockwise direction

        :param new_pos: A tuple of the new position of the robot's centre
        :param intersection_point: SPoint object for the intersection point
        :return: An angle in RADIANS
        """

        # Create a vector going from the centre to the nearest intersection point
        x1, y1 = new_pos
        x2, y2 = intersection_point.x, intersection_point.y
        direction_vector = [x2 - x1, y2 - y1]

        # This is an indication whether this vector points up or down. Since theta is measured clockwise, a vector
        # pointing up should return a positive angle. A vector pointing down means a negative angle
        shortest_line_points_up = y2 - y1 > 0

        # Normalise the vectors and compute the dot product
        unit_vector_1 = horizontal_vector / np.linalg.norm(horizontal_vector)
        unit_vector_2 = direction_vector / np.linalg.norm(direction_vector)
        dot_product = np.dot(unit_vector_1, unit_vector_2)

        angle = np.arccos(dot_product) if shortest_line_points_up else -np.arccos(dot_product)

        return angle

