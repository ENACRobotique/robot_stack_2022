
from sensor_msgs.msg import LaserScan
import math
from lidar_location.Point import Point
from lidar_location.Amalgame import Amalgame

# This variable defines the minimun distance to consider that 2 points are not part of the same Amalgame 
Amalgames_min_dist = 0.2 #meters


class Amalgame_list:
    def __init__(self, message):
        self.message = message
        self.list_points = self.message_to_points()
        self.points_length = len(self.list_points)
        self.list_obj = self.detect_Amalgames()
        """
        self.list_obj = []
        for pt in self.list_points:
            self.list_obj.append(Amalgame([pt]))
        """

    def message_to_points(self):
        list_points = []
        for index, distance in enumerate(self.message.ranges):
            new_point = Point()
            new_point.distance = distance
            new_point.set_angle(self.message.angle_increment, index)
            list_points.append(new_point)
        return list_points

    def detect_Amalgames(self):
        # First we'll need to get the first point of the list that is a break. This will allow us to work in a fully circular way
        start_position = self.find_first_break()
        position = start_position
        list_obj = []
        points_length = len(self.list_points)
        print("Pt depart: ", start_position)

        # Goes through all the list of points starting from the first break
        while position < points_length + start_position:
            list_pt_ama = [self.list_points[position % points_length]]
            print("BFEAK !")
            while (not self.is_break(position)) or(position < points_length + start_position) : ## TODO: CE TRUC EST BUGEE
                print("le break : ", self.is_break(position))
                print("le position : ", position)
                print("val max : ", points_length + start_position)
                position += 1
                list_pt_ama.append(self.list_points[position % points_length])

            #if(list_pt_ama[0].distance) > 0:
            list_obj.append(Amalgame(list_pt_ama))
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

        distance = abs(self.list_points[pos_first % self.points_length].distance - \
            self.list_points[(pos_first+1) % self.points_length].distance)

        if distance > Amalgames_min_dist:
            return True
        else:
            return False