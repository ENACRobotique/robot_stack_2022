from ctypes import sizeof
from sensor_msgs.msg import LaserScan
import math
from lidar_location.Point import Point
from lidar_location.Object import Object

# This variable defines the minimun distance to consider that 2 points are not part of the same object
objects_min_dist = 0.10


class Object_list:
    def __init__(self, message):
        self.list_obj = []
        self.list_points = []
        self.message = message
        self.detect_objects()

    def message_to_points(self):
        for index, distance in enumerate(self.message['ranges']):
            new_point = Point()
            new_point = distance
            new_point.set_angle(self.message['angle_increment'], index)
            self.list_points.append(new_point)

    def detect_objects(self):
        # First we'll need to get the first point of the list that is a break. This will allow us to work in a fully circular way
        start_position = self.find_first_break()
        position = start_position

        # Goes thoug all the list of points starting from the first break
        while position < sizeof(self.list_points) + start_position:
            list_pt_obj = [self.list_points[position]]
            # Creates a group of points while there's no break
            while (not self.is_break(position+1)) or (position < sizeof(self.list_points) + start_position):
                position += 1
                list_pt_obj.append(self.list_points[position])
            # at the end adds this list of points to an object
            self.list_obj.append(Object(list_pt_obj))

    # Returns the first point in the list that is a break
    # If not break is found, will return position 0

    def find_first_break(self):
        for index, point in enumerate(self.list_points):
            if self.is_break(point):
                return index

        return 0

    # gets last point before a break, returns true if this point is the last one before a break
    def is_break(self, pos_first):
        first_point = self.list_points[pos_first]
        second_point = self.list_points[0]
        if pos_first < sizeof(self.list_points):
            second_point = self.list_points[pos_first+1]

        angle_difference = abs(first_point.angle - second_point.angle)

        # Defines a and b to simplify calculations
        if second_point.distance - first_point.distance > 0:
            a = second_point
            b = first_point
        else:
            a = first_point
            b = second_point

        x1 = b.distance * math.cos(angle_difference)
        y1 = b.distance * math.sin(angle_difference)
        x2 = a.distance - x1

        pt_distance = math.sqrt(y1*y1 + x2*x2)

        return True if pt_distance > objects_min_dist else False
