from lidar_location.Amalgame_list import Amalgame_list
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
    triangle_pylon.angles = [1.265, 1.265, 0.612]
    triangle_pylon.distances = [1.9, 3.324, 3.324]
    # triangle_pylon.angles = [0.57, 1.19, 1.37]
    # triangle_pylon.distances = [1.83, 3.25, 3.85]

    # Defines the absolute location of the Pylons

    def __init__(self, message):
        self.obj_list = Amalgame_list(message)
        self.tri_list = self.create_triangle_list()
        self.valid_triangles = self.find_triangles()
        # self.location = self.find_location()

    def create_triangle_list(self):
        i = 0
        j = 1
        k = 2
        tri_list = []
        while i < len(self.obj_list.list_obj) - 2:
            j = i + 1
            while j < len(self.obj_list.list_obj) - 1:
                k = j + 1
                while k < len(self.obj_list.list_obj):
                    tri_list.append(Triangle(
                        self.obj_list.list_obj[i].relative_center, self.obj_list.list_obj[j].relative_center, self.obj_list.list_obj[k].relative_center))
                    k += 1
                j += 1
            i += 1
        return tri_list

    def find_triangles(self):
        valid_list = []
        for triangle in self.tri_list:
            # print("comparing", triangle.distances,
            #      "with", self.triangle_pylon.distances)
            if triangle.compare_triangles(self.triangle_pylon):
                valid_list.append(triangle)
        return valid_list

    # def find_location(self):
