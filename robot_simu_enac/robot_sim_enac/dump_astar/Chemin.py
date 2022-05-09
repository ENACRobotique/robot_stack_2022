
from Points import Points

class Chemin:
    def __init__(self,liste_Points=[]):
        self.liste_Points = liste_Points

    def add_Points(self,p:Points):
        self.liste_Points.append(p)
    
    def supp_Points(self,indice):
        del self.liste_Points[indice]

    def clear_chemin(self):
        self.liste_Points.clear()

    
    def afficher(self):
        i = 1
        for Points in self.liste_Points:
            print("Points",i,":")
            print("x:",Points.x)
            print("y:",Points.y)
            i+=1

p = Points(12,4)

p2 = Points(23,46)
p3 = Points(3,4)

liste = [p,p2,p3]

c = Chemin(liste)

c.afficher()
c.supp_Points(1)
print ("New affichage")
c.afficher()
        