from robot_nav_enac.Points import Points
import math

class Obstacle:

    def __init__(self):
        pass

    def IsInObstacle(self,p):
        raise


class Rond(Obstacle):
    def __init__(self,centre:Points,rayon):
        self.centre = centre
        self.rayon = rayon

    def distance(self,p2): 
        return math.sqrt((self.centre.x - p2.x)**2 + (self.centre.y - p2.y)**2)

    def IsInObstacle(self,p2):
        return (self.distance(p2)<=self.rayon)

    def __repr__(self) -> str:
        return "Rond(centre={},radius={})".format(self.centre, self.rayon)

class Rectangle(Obstacle):
    def __init__(self,a:Points,c:Points):
        self.a = a
        self.c = c
        x_b = c.x
        y_b = a.y
        self.b = Points(x_b,y_b)
        x_d = a.x
        y_d = c.y
        self.d = Points(x_d,y_d)

    def IsInObstacle(self,p):
        return ((p.x<self.b.x) & (p.x>self.a.x) & (p.y<self.d.y) & (p.y>self.a.y)) 

    def __repr__(self) -> str:
        return "Rectangle(A={},B={},C={},D={})".format(self.a, self.b,self.c,self.d)
        #return "Rectangle"

class Coin(Obstacle):
    def __init__(self):
        pass
        
 
    def IsInObstacle(self,p):
        #y = x - 290
        return ((p.x+p.y)<51 | (p.x-p.y>290))

    def __repr__(self) -> str:
        return "Coin"


class Obstacle_Mobile(Obstacle):
      def __init__(self,position,rayon,deplacement):
        self.position = position
        self.rayon = rayon
        self.deplacement = deplacement

