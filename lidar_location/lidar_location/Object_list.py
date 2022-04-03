
from sensor_msgs.msg import LaserScan
import math
from lidar_location.Point import Point
from lidar_location.Object import Object

# This variable defines the minimun distance to consider that 2 points are not part of the same object
objects_min_dist = 0.0


class Object_list:
    def __init__(self, message):
        self.message = message
        self.list_points = self.message_to_points()
        self.list_obj = self.detect_objects()

    def message_to_points(self):
        list_points = []
        for index, distance in enumerate(self.message.ranges):
            new_point = Point()
            new_point.distance = distance
            new_point.set_angle(self.message.angle_increment, index)
            list_points.append(new_point)
        return list_points

    def detect_objects(self):
        # First we'll need to get the first point of the list that is a break. This will allow us to work in a fully circular way
        start_position = self.find_first_break()
        position = start_position
        list_obj = []
        points_length = len(self.list_points)

        # Goes through all the list of points starting from the first break
        while position < points_length + start_position:
            if self.list_points[position%points_length] > 0 :
                pos_temp = position
                list_pt_obj = [self.list_points[pos_temp%points_length]]
                # Creates a group of points while there's no break
                while (not self.is_break(pos_temp)) or (pos_temp < points_length + start_position):
                    pos_temp += 1
                    list_pt_obj.append(self.list_points[pos_temp%points_length])
                # at the end adds this list of points to an object
                #if list_pt_obj[0].distance > 0:  # Removes filtered points
                list_obj.append(Object(list_pt_obj))
            position += 1
        return list_obj

    # Returns the first point in the list that is a break
    # If not break is found, will return position 0

    def find_first_break(self):
        for index, point in enumerate(self.list_points):
            if self.is_break(index):
                return index

        return 0

    # gets last point before a break, returns true if this point is the last one before a break
    def is_break(self, pos_first):
        # The break calculation hasd been simplified from a previous complex calculation.
        # Given the small angle between 2 points (less than 1 degree) we can assume that points
        # are aligned and thus calculate only the difference in distance between 2 points
        points_length = len(self.list_points)

        distance = self.list_points[pos_first%points_length].distance - self.list_points[(pos_first+1)%points_length].distance

        return True if distance > objects_min_dist else False
