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

        # Recursive check for legal position
        legal = False
        while not legal:
            # new_pos, legal = self.correct_pos(new_pos)
            new_pos, legal = self.correct_pos2(old_pos, new_pos, new_theta)

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

    def correct_pos2(self, old_pos, new_pos, new_theta):
        """
        A more robust collision detection method

        First we check if the robot is currently crossing the line:
        If it is, the robot's circle will either have two or one intersection(s) point(s) with a wall.
        In the case of two points, we use the mid point. In case of one point per wall, we use the point itself.

        With all found points we compute the minimum Euclidean distance from the intersections and the robot's
        previous centre

        Let the smallest Euclidean distance between the robot's previous centre and an intersection point be called D.
        Let theta be the robot's angle of orientation after its position has been updated. Let R be the radius of the
        robot. And finally, let the robot's previous centre be x, y

        The robot's new position x', y' will be:

        x' = x + D * Cos(theta) - R
        y' = y + D * Cos(theta) - R

        Second, if the robot is too fast and fully crosses the line, then we check for the intersection of the line
        connecting its old and new centre and a wall. The same formula applies

        :param old_pos: Tuple of old position (x_old, y_old)
        :param new_pos: Tuple of new position (x_new, y_new)
        :param new_theta: Angle in degrees of robot's orientation in its new position
        :return: Returns a new position for the robot (x', y')
        """

        old_centre = Point(old_pos[0], old_pos[1])
        new_centre = Point(new_pos[0], new_pos[1])
        robot_circle = Ellipse(center=new_centre, hradius=robot_radius, vradius=robot_radius)

        travelled_path = Segment(old_centre, new_centre)

        circle_intersection_distances = []
        line_intersections_distances = []

        for wall in self.room_map:
            circle_intersection_points = robot_circle.intersection(wall)
            line_intersection_points = travelled_path.intersection(wall)

            # Do a preliminary check to see if there are any intersections first, to avoid useless calculations
            if len(circle_intersection_points) < 1 and len(line_intersection_points) < 1:
                continue

            # Does the circle intersect at two points on the same wall? --> midpoint
            if len(circle_intersection_points) == 2:
                mid_point = circle_intersection_points[0].midpoint(circle_intersection_points[1])

                circle_intersection_distances.append(old_centre.distance(mid_point))

                # This info is enough to correct the position
                continue

            # Does the circle intersect just one point on the wall? Add it as a single point
            if len(circle_intersection_points) == 1:
                circle_intersection_distances.append(old_centre.distance(circle_intersection_points[0]))

                # This info is enough to correct the position
                continue

            # If no circle intersections, there could be line intersections
            for line_intersection_point in line_intersection_points:
                line_intersections_distances.append(old_centre.distance(line_intersection_point))

        # If no intersections with all walls, just return new pos
        if len(circle_intersection_distances) < 1 and len(line_intersections_distances) < 1:
            return new_pos, True

        # First work with circle intersections, they're the easiest
        if len(circle_intersection_distances) > 0:
            min_distance = min(circle_intersection_distances)

            return self.get_corrected_xy(old_pos[0], old_pos[1], min_distance, new_theta, robot_radius), True

        # If no circle intersections, then the line intersections are the only option
        min_distance = min(line_intersections_distances)

        return self.get_corrected_xy(old_pos[0], old_pos[1], min_distance, new_theta, robot_radius), True

    def get_corrected_xy(self, x, y, min_distance, new_theta, radius):
        """
        Returns the corrected x and y position of the robot

        :param x: Robot's old x coord
        :param y: Robot's old y coord
        :param min_distance: Min distance between old centre and closest intersection point
        :param new_theta: Robot's new angle of orientation after position update
        :param radius: Radius of the robot
        :return: Returns a tuple with the new x and y coord of the robot
        """

        return x + (min_distance - radius) * np.cos(np.radians(new_theta)), \
               y + (min_distance - radius) * np.sin(np.radians(new_theta))


