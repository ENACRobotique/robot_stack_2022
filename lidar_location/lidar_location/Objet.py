import Point

types_obj = ['Pylon', 'Tower', 'Friend', 'Foe', 'Unknown']


class Objet:
    def __init__(self, typeobj):
        self.type = typeobj
        self.liste_points
        self.point_debut
        self.point_fin
        self.relative_position
        self.absolute_position

    def set_point_debut(self, increment, position):
        self.point_debut = Point()
        self.point_debut.set_angle(increment, position)

    def set_point_fin(self, increment, position):
        self.point_fin = Point()
        self.point_fin.set_angle(increment, position)
