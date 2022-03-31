
from ctypes import sizeof
from lidar_location.Object_list import Object_list
from lidar_location.Triangle import Triangle
from lidar_location.Point import Point


class Triangulation:
    # Create abasic pylon triangle and sets its parameters
    pt1 = Point()
    pt1.angle = 0.1
    pt1.distance = 1
    pt2 = Point()
    pt2.angle = 0.2
    pt2.distance = 2
    pt3 = Point()
    pt3.angle = 0.3
    pt3.distance = 3
    triangle_pylon = Triangle(pt1, pt2, pt3)
    triangle_pylon.angles = [1.21, 1.21, 0.72]
    triangle_pylon.distances = [1.9, 3.2, 3.2]

    def __init__(self, message):
        self.obj_list = Object_list(message)
        self.tri_list = self.create_triangle_list()
        self.valid_triangles = self.find_triangles()

    def create_triangle_list(self):
        i = 0
        j = 1
        k = 2
        tri_list = []
        while i < sizeof(self.obj_list.list_obj) - 2 and j < sizeof(self.obj_list.list_obj) - 1 and k < sizeof(self.obj_list.list_obj):
            tri_list.append(Triangle(
                self.obj_list.list_obj[i], self.obj_list.list_obj[j], self.obj_list.list_obj[k]))
        return tri_list

    def find_triangles(self):
        valid_list = []
        for triangle in self.tri_list:
            if triangle.compare_triangles(self.triangle_pylon):
                valid_list.append(triangle)
        return valid_list
