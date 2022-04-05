
class Point:
    def __init__(self):
        self.angle = 0  # Radians
        self.distance = 0  # en m
        self.pos_x = 0  # Position de l'element une fois determine
        self.pos_y = 0

    # Retourne l'angle relatif du point en fonction de sa position et le pas de l'angle
    def set_angle(self, increment, position):
        # Increment represente l'increment en radians de chaque point
        self.angle = increment * position

    def get_position_rel(self):
        return [self.angle, self.distance]

    # def set_abs_pos(self, x, y)
