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
        old_pos = self.pos

        # Get new prospective position
        new_pos_orient = self.get_new_pos()
        new_theta = new_pos_orient[2]
        new_pos = (new_pos_orient[0], new_pos_orient[1])

        legal = False
        while not legal:
            new_pos, legal = self.correct_pos2(old_pos, new_pos)
        print(f"final pos: {new_pos}")
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
        Corrects the new centre of the robot the following way:

        First we check for circle intersections. If there's an intersection with one wall, there will be two points
        of contact. The mid point, m, of these two points of contact is the point that is the shortest distance away
        from the centre of the robot. Let the line that joins the centre of the robot and m, be called l. Let angle
        of inclination of l wrt to the positive x-axis be called alpha. The amount that the robot must slide in the
        direction is moves is a constant component/fraction of its radius. In this case, the constant is set to 2.

        Therefore the final corrected position of the robot will be:

        new_pos = new_pos + pos_offset + sliding_component

        Where the offsets are given by:

        x_offset = -R * np.cos(alpha)
        y_offset = -R * np.sin(alpha)

        And the sliding components are:

        x_slide = average_velocity * np.cos(theta)
        y_slide = average_velocity * np.sin(theta)

        If the circle intersects two walls (a corner where the robot is trapped within), it will take the average
        of the two suggested positions

        In the case that the robot fully crosses a wall, a line is drawn between the centres of the two positions of the
        robot. The above equations apply the same way where m is the point of intersection where l2 meets the closest
        wall

        :param old_pos: Tuple indicating old position
        :param new_pos: Tuple indicating new position
        :return: Tuple with corrected position
        """

        old_centre = SPoint(old_pos)
        new_centre = SPoint(new_pos)
        robot_circle = new_centre.buffer(robot_radius).boundary

        travelled_path = LineString([old_pos, new_pos])

        circle_distances = {}
        line_distances = {}

        for wall in self.room_map:
            circle_intersection_points = robot_circle.intersection(wall)
            line_intersection_points = travelled_path.intersection(wall)

            # Do a preliminary check to see if there are any intersections first, to avoid useless calculations
            if circle_intersection_points.is_empty and line_intersection_points.is_empty:
                continue

            # Does the circle intersect at two points on the same wall? --> midpoint
            if type(circle_intersection_points) is MultiPoint:

                if len(circle_intersection_points.geoms) == 2:
                    p1 = SPoint((circle_intersection_points.geoms[0].x, circle_intersection_points.geoms[0].y))
                    p2 = SPoint((circle_intersection_points.geoms[1].x, circle_intersection_points.geoms[1].y))

                    mid_x = (p1.x + p2.x)/2
                    mid_y = (p1.y + p2.y)/2

                    mid_point = SPoint((mid_x, mid_y))
                    distance_to_point = new_centre.distance(mid_point)
                    circle_distances[distance_to_point] = mid_point, wall

                    # This info is enough to correct the position
                    continue

            # Does the circle intersect just one point on the wall? Add it as a single point
            if type(circle_intersection_points) == SPoint:

                distance_to_point = new_centre.distance(circle_intersection_points)
                circle_distances[distance_to_point] = circle_intersection_points, wall

                # This info is enough to correct the position
                continue

            # If no circle intersections, there could be line intersections
            if line_intersection_points.is_empty:
                continue

            # If a single point
            if type(line_intersection_points) == SPoint:

                distance_to_point = old_centre.distance(line_intersection_points)
                line_distances[distance_to_point] = line_intersection_points, wall

            # If multiple points - very rare case
            if type(line_intersection_points) == MultiPoint:

                for i in range(len(line_intersection_points.geoms)):

                    distance_to_point = old_centre.distance(line_intersection_points.geoms[i])
                    line_distances[distance_to_point] = line_intersection_points.geoms[i], wall

        # If no intersections with all walls, just return new pos
        if len(circle_distances) < 1 and len(line_distances) < 1:
            return new_pos, True

        # First check circle possible circle intersections
        if len(circle_distances) > 0:

            distances = list(circle_distances.keys())

            minimum_distance = min(circle_distances.keys())
            closest_point, wall = circle_distances[minimum_distance]

            normal_line = self.get_normal_vector(wall)
            normal_line_inclination = self.get_normal_line_inclination(normal_line)

            return self.get_corrected_xy(closest_point, normal_line_inclination), True

            # # If it collides with two or more walls, we want the two closest walls
            # if len(distances) > 1:
            #
            #     distances.sort()
            #     p1, wall1 = circle_distances[distances[0]]
            #     p2, wall2 = circle_distances[distances[1]]
            #
            #     points = [p1, p2]
            #     walls = [wall1, wall2]
            #     x, y = [], []
            #
            #     # The new position will be the average of both walls' corrections
            #     for point, wall in zip(points, walls):
            #         normal_line = self.get_normal_vector(wall)
            #         normal_line_inclination = self.get_normal_line_inclination(normal_line)
            #         est_x, est_y = self.get_corrected_xy(point, normal_line_inclination)
            #         x.append(est_x)
            #         y.append(est_y)
            #
            #     return (np.mean(x), np.mean(y)), True

        # The only remaining case is line intersections
        minimum_distance = min(line_distances.keys())
        closest_point, wall = line_distances[minimum_distance]

        normal_line = self.get_normal_vector(wall)
        normal_line_inclination = self.get_normal_line_inclination(normal_line)

        return self.get_corrected_xy(closest_point, normal_line_inclination), True

    def get_corrected_xy(self, intersection_point, normal_line_inclination):
        """
        Gets the corrected centre position of the robot

        :param normal_line_inclination:
        :param intersection_point: SPoint object indicating the closest intersection point
        intersection point
        :return: A tuple of integers indicating the robot's new centre (x, y)
        """

        x, y = intersection_point.x, intersection_point.y

        # Here we calculate the offset of intersection point, so that the new centre lies a radius away from the
        # intersection point
        x_offset = (robot_radius + robot_border_size) * np.cos(normal_line_inclination)
        y_offset = -(robot_radius + robot_border_size) * np.sin(normal_line_inclination)

        # This will be the new centre, but we need to induce some sliding
        new_x, new_y = x + x_offset, y + y_offset

        return new_x, new_y

    def get_normal_vector(self, wall):

        x1, y1 = wall.coords[0][0], wall.coords[0][1]
        x2, y2 = wall.coords[1][0], wall.coords[1][1]
        print(x1, x2, y1, y2)
        normal_vector = [-(y1 - y2), x1 - x2]
        print(normal_vector)
        return normal_vector / np.linalg.norm(normal_vector)

    def get_normal_line_inclination(self, normal_vector):

        # This is an indication whether this vector points up or down. Since theta is measured clockwise, a vector
        # pointing up should return a positive angle. A vector pointing down means a negative angle
        shortest_line_points_up = normal_vector[1] < 0
        print(shortest_line_points_up)
        # Normalise the vectors and compute the dot product
        unit_vector_1 = horizontal_vector / np.linalg.norm(horizontal_vector)

        dot_product = np.dot(unit_vector_1, normal_vector)

        angle = np.arccos(dot_product) if shortest_line_points_up else -np.arccos(dot_product)
        print(np.degrees(angle))
        return angle
