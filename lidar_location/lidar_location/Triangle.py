import lidar_location.Point as Point
import math

# Delta angle of approximation between 2 segments
jitter_angle = 0.0872  # 0.0872 = 5 degrees

# Delta of approximation in terms of distance
jitter_distance = 0.10  # 10 centimeters


class Triangle:
    def __init__(self, pt1, pt2, pt3):
        self.pt_list = [pt1, pt2, pt3]
        self.angles = self.get_angles()
        self.distances = self.get_distances()

    # Returns distances between [pt1 and pt2, pt2 and pt3, pt3 and pt1]
    def get_distances(self):
        return [self.get_distance_pt(self.pt_list[0], self.pt_list[1]), self.get_distance_pt(self.pt_list[1], self.pt_list[2]), self.get_distance_pt(self.pt_list[2], self.pt_list[0])]

        # Returns angles between segments a-b/b-c [pt1-pt2/pt2-pt3, pt2-pt3/pt3-pt1, pt3-pt1/pt1-pt2]
    def get_angles(self):
        angle_difference = abs(self.pt_list[0].angle - self.pt_list[1].angle)

        # Defines a and b to simplify calculations
        if self.pt_list[0].distance - self.pt_list[1].distance > 0:
            a = self.pt_list[1]
            b = self.pt_list[0]
        else:
            a = self.pt_list[0]
            b = self.pt_list[1]

        x1 = b.distance * math.cos(angle_difference)
        y1 = b.distance * math.sin(angle_difference)
        x2 = a.distance - x1
        # Reusing code from Object_list.is_break
        y3 = self.pt_list[1].distance * \
            math.cos(math.pi - self.pt_list[1].angle)

        a1 = math.atan(y1/x2)
        a2 = math.pi/2 - a1
        a3 = math.pi/2 - self.pt_list[1] - self.pt_list[0]
        a4 = math.asin(y3/(self.pt_list[1].distance))

        angle_pt2 = a2 + a3 + a4

        a5 = math.asin(y3/(self.pt_list[2].distance))
        a6 = math.pi - \
            (2 * math.pi - self.pt_list[2].angle) + self.pt_list[0].angle + a1

        angle_pt3 = a6 + a5

        angle_pt1 = math.pi - angle_pt2 - angle_pt3

        return [angle_pt1, angle_pt2, angle_pt3]

    # Returns true if triangle fits within approximation values (distance-angle)
    def compare_triangles(self, triangle2):
        if self.compare_angles(triangle2) and self.compare_distances(triangle2):
            return True
        else:
            return False

    def compare_angles(self, triangle2):
        for angle1 in self.angles:
            is_triangle = False
            for angle2 in triangle2.angles:
                is_triangle = True
                if math.abs(angle1 - angle2) > jitter_angle:
                    is_triangle = False
                if is_triangle == True:
                    return True
        return False

    def compare_distances(self, triangle2):
        for dist1 in self.distances:
            for dist2 in triangle2.distances:
                is_triangle = True
                if math.abs(dist1 - dist2) > jitter_distance:
                    is_triangle = False
                if is_triangle == True:
                    return True
        return False

    def get_distance_pt(self, pt1, pt2):
        angle_difference = abs(pt1.angle - pt2.angle)

        # Defines a and b to simplify calculations
        if pt2.distance - pt1.distance > 0:
            a = pt2
            b = pt1
        else:
            a = pt1
            b = pt2

        x1 = a * b.distance * math.cos(angle_difference)
        y1 = b.distance * math.sin(angle_difference)
        x2 = a.distance - x1

        return math.sqrt(y1*y1 + x2*x2)
