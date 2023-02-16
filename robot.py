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
            pass

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

        return []

    def update_position(self):
        pass
        # TODO: Implement function to update the Robot position based on current position,
        # wheel velocities and environment

        # Remember that in the edge case: we have to recursively update position
