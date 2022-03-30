from ctypes import sizeof
from sensor_msgs.msg import LaserScan
import Point


class Object_list:
    def __init__(self, message):
        self.list_obj
        self.list_points
        self.message = message

    def message_to_points(self):
        for index, distance in enumerate(self.message['ranges']):
            new_point = Point()
            new_point = distance
            new_point.set_angle(self.message['angle_increment'], index)
            self.list_points.append(new_point)

    def detect_objects(self):
        position = find_first_break()

    def find_first_break(self):
        while()

    # gets last point before a break, returns true if this point is the last one before a break
    def is_break(self, pos_first):
        first_point = self.list_points[pos_first]
        if pos_first < sizeof(self.list_points):
            second_point = self.list_points[pos_first+1]
        else
        second_point = self.list_points[0]

        angle_difference = abs(first_point.angle - second_point.angle)

        # Defines a and b to simplify calculations
        if second_point.distance - first_point.distance > 0:
            a = second_point
            b = first_point
        else:
            a = first_point
            b = second_point
