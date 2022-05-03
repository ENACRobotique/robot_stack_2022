import math

class Point:
    def __init__(self):
        self.angle = 0  # Radians
        self.distance = 0  # en m
        self.pos_x = 0  # Position de l'element une fois determine
        self.pos_y = 0
        self.set_abs_pos_robot()

    # Retourne l'angle relatif du point en fonction de sa position et le pas de l'angle
    def set_angle(self, increment, position):
        # Increment represente l'increment en radians de chaque point
        self.angle = increment * position

    # Retourne l'angle relatif du point 
    def get_angle(self):
        return self.angle

    def get_position_rel(self):
        return [self.angle, self.distance]

    def set_abs_pos_robot(self):
        self.pos_x = self.distance * math.cos(self.angle)
        self.pos_y = self.distance * math.sin(self.angle)

