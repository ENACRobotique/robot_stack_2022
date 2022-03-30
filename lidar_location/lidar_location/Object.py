from ctypes import sizeof
from lidar_location.Point import Point

types_obj = ['Pylon', 'Tower', 'Friend', 'Foe', 'Unknown']


class Object:
    def __init__(self, list_points):
        self.type
        self.list_points = list_points
        self.relative_center = self._calculate_relative_center()
        self.absolute_position
        self.size

    def _calculate_relative_center(self):
        mean_distance = 0
        for point in self.list_points:
            mean_distance += point.distance
        mean_distance /= sizeof(self.list_points)
        mean_angle = self.list_points[sizeof(self.list_points)/2].angle

        return [mean_distance, mean_angle]
