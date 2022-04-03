
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
        mean_distance = 0
        new_p = Point()
        for point in self.list_points:
            mean_distance += point.distance
        new_p.distance /= len(self.list_points)
        new_p.angle = self.list_points[math.floor(len(self.list_points)/2)].angle

        return new_p
