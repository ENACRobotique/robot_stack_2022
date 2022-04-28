
from lidar_location.Point import Point
import math

types_obj = ['Pylon', 'Tower', 'Friend', 'Foe', 'Unknown']

angle_increment = 0.013877294957637787


class Amalgame:
    def __init__(self, list_points):
        self.type = ""
        self.list_points = list_points
        self.relative_center = self._calculate_relative_center()
        self.absolute_position = []
        self.size = self.get_size()

    def _calculate_relative_center(self):
        new_p = Point()
        new_p.distance = 0

        for point in self.list_points:
            new_p.distance = new_p.distance + point.distance

        new_p.distance = new_p.distance / (len(self.list_points))
        new_p.angle = self.list_points[math.floor(
            len(self.list_points)/2)].angle
        new_p.set_abs_pos_robot()

        return new_p

    def get_size(self):
        return 2 * self.relative_center.distance * math.tan(((len(self.list_points) - 1) * angle_increment) / 2)
