
from lidar_location.Point import Point
import math

types_obj = ['Pylon', 'Tower', 'Friend', 'Foe', 'Unknown']


class Object:
    def __init__(self, list_points):
        self.type = ""
        self.list_points = list_points
        self.relative_center = self._calculate_relative_center()
        self.absolute_position = []
        self.size = 0

    def _calculate_relative_center(self):
        new_p = Point()
        new_p.distance = 0
        new_p.angle = 0

        for point in self.list_points:
            new_p.distance = new_p.distance + point.distance
            new_p.angle = new_p.angle + point.angle

        new_p.distance = new_p.distance / len(self.list_points)
        new_p.angle = new_p.angle / len(self.list_points)

        return new_p
