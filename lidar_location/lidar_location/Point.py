
class Point:
    def __init__(self):
        self.angle  # Radians
        self.distance  # en m
        self.position_absolue  # Position de l'element une fois determine

    # Retourne l'angle relatif du point en fonction de sa position et le pas de l'angle
    def set_angle(self, increment, position):
        # Increment represente l'increment en radians de chaque point
        self.angle = increment * position

    def get_position_rel(self):
        return [self.angle, self.distance]
