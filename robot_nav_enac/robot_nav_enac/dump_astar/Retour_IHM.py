from Sauvegarde.Obstacle_v1 import *
from Points import Points
def Recuperation_IHM(liste_rectangle,liste_rond):
    liste_obstacle = []
    #for rectangle in liste_rectangle:
    #    liste_obstacle.append(rectangle)
    i = 0
    while(i<len(liste_rond)):
        r = Rond(Points(liste_rond[i][0]*rapport_x,liste_rond[i][1]*rapport_y),liste_rond[i+1])
        liste_obstacle.append(r)
        i = i+2

   # for rond in liste_rond:
    #    liste_obstacle.append(rond)

    return liste_obstacle

rapport_x = 1/((1288/(3000))*10)
rapport_y = 1/((858/(2000))*10)


liste = [Points(x=916, y=225), 56.568542494923804, Points(x=867, y=350), 56.568542494923804,
         Points(x=948, y=467), 56.568542494923804, Points(x=822, y=460), 56.568542494923804]

liste_1 = [Points(x=1283, y=855), 56.568542494923804, Points(x=5, y=5), 56.568542494923804]

a = Recuperation_IHM([],liste_1)

print(a)

