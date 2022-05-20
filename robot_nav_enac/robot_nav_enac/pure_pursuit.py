# COLL SACHA pursuit
import math
#from winreg import QueryReflectionKey
import numpy as np
import numpy as numpy
import matplotlib.pyplot as plt
from robot_nav_enac.Points import Points
from robot_nav_enac.conversions import z_euler_from_quaternions, quaternion_from_euler

# Vitesse de Target :
Vtarget = 0.1 # k

# Distance entre Target et Robot :
Dtrob = 1.0   # Lfc = 1.0

# Gain Vitesse pour régler rampe d'accélération
# Kp : https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
Krampe = 2.0 # Kp = 1.0

Kyaw = 0.5

# Dt [s] ????
dt = 0.1 

# Empattement robot [m]
L = 2.9

list_detect = [] # Stabilisation des changements de vitesse
liste_waypoint = [] # Liste Waypoint
liste_astar = []

show_animation = True

class State:

    # Initialisation position
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x          # Xrobot
        self.y = y          # Yrobot
        self.yaw = yaw      # Angle de rotation du Robot https://automaticaddison.com/yaw-pitch-and-roll-diagrams-using-2d-coordinate-systems/
                            #                            https://en.wikipedia.org/wiki/Yaw_(rotation)
        self.v = v          # Vitesse 
        self.dyaw = 0.0
        liste_waypoint.append(Points(47.29,-0.74)) # Point central
        liste_waypoint.append(47)
        # self.vang = vang

    def __repr__(self) -> str:
        return "x = {} y = {} yaw = {} v = {} dyaw = {} ".format(self.x, self.y, self.yaw, self.v, self.dyaw)


def InObjectif(self):
    global IsInObjectif
    #print("CECI EST ISINOBJECTIF")
    #print(IsInObjectif)
    i = 0 
    while i < len(liste_waypoint):
        pointest = liste_waypoint[i]
        ray = liste_waypoint[i+1]
        i += 2
        dist = math.sqrt((pointest.x - self.x)**2 + (pointest.y - self.y)**2)
        xmin = pointest.x-4
        xmax = pointest.x+4
        ymin = pointest.y-4
        ymax = pointest.y+4
        if (dist <= ray) and ((self.x > xmin) and (self.x < xmax) and (self.y > ymin) and (self.y < ymax)):
            IsInObjectif = True
        else:
            IsInObjectif = False


    # Met à jour la position du robot (X;Y;Yaw;V)
def update(state, a, delta):
    global detect
    global IsInObjectif
    global tryagain
    global target_speed
    global cpt
    global arret
    global arret2
    # Calcul de X et Y p.6 : https://homes.cs.washington.edu/~todorov/courses/cseP590/05_Kinematics.pdf
    state.x = state.x + state.v * math.cos(state.yaw) * dt      # Update Position X
    state.y = state.y + state.v * math.sin(state.yaw) * dt      # Update Position Y
    InObjectif(state)
    #print(state.v / L * math.tan(delta) * dt)
    state.dyaw = state.v / L * math.tan(delta) * dt
    #print(tryagain)
    #print(cpt)




    if ((detect<0 and detect - state.dyaw > 0) or (detect>0 and detect - state.dyaw < 0)) and (state.dyaw>0.01 or state.dyaw < -0.01) : # Si changement de signe de vitesse angulaire
                                        #<                                        #>
         #state.v = state.v - a * dt/2
         # and ((state.dyaw < -0.00) or (state.dyaw > 0.00))
        #tryagain = True
        cpt += 1
        if cpt<20: # cpt<30 60 100
            tryagain = True
        else:
            exit
    else: 
         #state.v = state.v + a * dt
        tryagain = False
        cpt = 0

    if tryagain or arret:
        target_speed = 0.2 #0.3
        #cpt = 0
    elif IsInObjectif:
        target_speed = 0.1 #0.1
    elif arret:
        target_speed = 0.0 #0.3
    elif arret2:
        target_speed = 0.0
    else:
        target_speed = 0.3 # 0.6
    
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt  # Update Angle Rotation Yaw
    state.v = state.v + a * dt                                  # Update Vitesse
    #state.vang = state.vang + newvang
    detect = state.v / L * math.tan(delta) * dt
    return state

    # Variation du rapport cyclique du moteur
def PIDControl(target, current):         # Proportional Integral Derivative => Vcurrent est mesurée et comparée avec vitesse voulue
    a = Krampe * (target - current)      # Explication de cette formule : https://www.sciencedirect.com/topics/computer-science/proportional-controller

    return a

    # Calcul Point de passage le plus proche du véhicule => Trouve l'indicateur du point
def calc_target_index(state, cx, cy): # Calcul l'indicateur

    # Recherche de l'index du point le plus proche
    dx = [state.x - icx for icx in cx]      # dx = Xrobot - Xchemin
    dy = [state.y - icy for icy in cy]      # dy = Yrobot - Ychemin
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]     # Distance Robot(x;y)-> FuturTarget(x;y)
    ind = d.index(min(d))       # Récupère index de la distance la plus petite
    L = 0.0                     # Init de L

    Lf = Vtarget * state.v + Dtrob

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2) # L = L + ds
        ind += 1
############ RAJOUTER MODIF DE LA VITESSE  + MODIF Vangulaire
    return ind

def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy) # Récupère l'index du prochain point

    if pind >= ind:                        # Si indicateur position robot >= indicateur prochain point
        ind = pind                         # On met à jour l'indicateur du prochain point

    if ind < len(cx):                      # Si il reste des points du chemin
        tx = cx[ind]                       #Target(x = Xcheminprochainpoint ; y = Ycheminprochainpoint)
        ty = cy[ind]
    if (ind > len(cx)-30) and (ind < len(cx)):
        tx = cx[ind]                       #Target(x = Xcheminprochainpoint ; y = Ycheminprochainpoint)
        ty = cy[ind]
        arret = True
        arret2 = False
    if (ind > len(cx)-15) and (ind < len(cx)):
        tx = cx[ind]                       #Target(x = Xcheminprochainpoint ; y = Ycheminprochainpoint)
        ty = cy[ind]
        arret = False
        arret2 = True
    else:
        tx = cx[-1]                        # Sinon Target(x = Xcheminprochainpoint-1 ; y = Ycheminprochainpoint-1)
        ty = cy[-1]                        ############################ A modifier si on veut arriver à l'arrivée
        ind = len(cx) - 1                  # Met à jour l'indicateur du prochain point

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = Vtarget * state.v + Dtrob

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind

target_speed = 0
cpt = 0




def Pure_poursuit(chemin_astar,angle_init):
     ## CREATION CX ET CY QUI CORRESPONDENT AUX POINTS DU CHEMIN DE L'ASTAR
    cx = np.array([])
    cy = np.array([])
    #print(chemin_astar)
    print("Purepoursuit called (x={}, y={})".format(chemin_astar[0].x, chemin_astar[0].y))


    ## REMPLISSAGE DE CX ET CY VIA LA LISTE GÉNÉRÉE PAR L'ASTAR QUI CONTIENT LES POINTS DU CHEMIN À SUIVRE
    n = 0 
    while n < min(len(chemin_astar), 10):
        cx = numpy.append(cx,chemin_astar[n].x)
        cy = numpy.append(cy,chemin_astar[n].y)
        #cx = numpy.append(cx,5.0)
        #cy = numpy.append(cy,5.0)
        if n == 0:
            state = State(x=chemin_astar[n].x, y=chemin_astar[n].y, yaw=angle_init, v=0.0)
        n += 1

    #cx = np.arange(0, 60, 0.1) # (Xdebut, Xarrivée, Step)
    #cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    #cx = np.array([4.0 ,5.0 ,6.0 ,7.0 ,8.0 ,9.0 ,10.0 ,11.0 ,15.0 ,16.0]) # ABSCISSE DE CHAQUE POINT DU CHEMIN 
    #cy = np.array([0.0 ,1.0 ,2.0 ,3.0 ,4.0 ,5.0 ,5.0 ,6.0 ,7.0 ,8.0]) # ORDONNÉE DE CHAQUE POINT DU CHEMIN

    # RÉGLER VITESSE
    #target_speed = 10.0 / 3.6  # [m/s]

    # TEMPS MAX SIMULATION
    T = 100.0 


    # Dernier Index = Index du Dernier Point - 1
    lastIndex = len(cx) - 1
    time = 0.0

    # Récupère Infos du Robot
    x = [state.x] 
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    dyaw = [0.0]

    # Index de la Cible 
    target_ind = calc_target_index(state, cx, cy)

    # Tant que Tsimu>0 ou Point d'Arrivée Pas Passé
    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)                           # Récupère le coef d'accélération ( PID )
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind) # Récupère Delta et l'Indicateur à jour
        state = update(state, ai, di)                                    # Met à jour infos robot

        time = time + dt                                                 # Met à jour temps

        # Init des Listes des infos du robot
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        dyaw.append(state.dyaw)

    liste_vitesse = []
    linear_speed = v
    angular_speed = state.yaw
    liste_vitesse = [linear_speed, angular_speed]
    print(angular_speed)
    print("state",state)

    return(liste_vitesse)




################################### RESTANT ###################################
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:
        plt.plot(cx, cy, ".r", label="course") # CHEMIN GÉNÉRÉ PAR A*
        plt.plot(x, y, "-b", label="trajectory") # TRAJECTOIRE EMPRUNTÉ PAR ROBOT
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        #plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.plot(t, [iv for iv in dyaw], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

arret2 = False
arret = False
IsInObjectif = False
tryagain = False
detect = 0

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    chemin_astar=[Points(20,20),Points(30,30),Points(40,40),Points(50,50)]
    Pure_poursuit(chemin_astar)