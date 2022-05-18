from robot_nav_enac.NavigationType import NavigationType, OdomData

from robot_nav_enac.pure_pursuit import Pure_poursuit
import  robot_nav_enac.Astar_V4_S as astar
from robot_nav_enac.Obstacle import Obstacle, Rond, Rectangle,Coin
from robot_nav_enac.Points import Points

class PurePursuit(NavigationType):
    def __init__(self):
        self.fixed_obstacle = self.Map_init()
        self.target = OdomData(-1,-1,-1)
        pass

    def Map_init(self):
        c = Coin()
        liste_obs = []

        r1 = Rectangle(Points(45,0),Points(117,8.5))
        r2 = Rectangle(Points(127.5,0),Points(142.5,10.2))
        barre = Rond(Points(150, 5), 5)
        moitie = 1500
        r2_bis = Rectangle(Points((moitie+75)/10,0),Points((moitie+225)/10,10.2))
        r1_bis = Rectangle(Points((moitie+330)/10,0),Points((moitie+1050)/10,8.5))
        liste_obs.append(r1)
        liste_obs.append(r2)
        liste_obs.append(r1_bis)
        liste_obs.append(r2_bis)
        #liste_obs.append(barre)

        return liste_obs # return liste_obs apres debugage

    def update_dyn_obstacles(self, dynamic_obstacles = None):
        
        pass
    
    def set_target(self, target_pose):
        self.target = target_pose
        # astar with dynamic obstacles in parameters
        #self.path_to_follow = result_from_astar
        pass

    def update_odom(self, callback_speed, position, speed, dt):

        #finding astar path
        liste_obstacle = self.fixed_obstacle
        graph = astar.AStarGraph(liste_obstacle)
        meter_to_astar_unit = 100 #convert to cm
        depart = Points(int(position.x * meter_to_astar_unit), int(position.y * meter_to_astar_unit))
        arrive = Points(int(self.target.x * meter_to_astar_unit), int(self.target.y * meter_to_astar_unit))
        path_to_follow, cost = astar.AStarSearch(depart, arrive, graph)
        # generate cons_speed from pure pursuit algorithm
        speeds = Pure_poursuit(path_to_follow, position.rotation_rad)
        index_target = 10 #index of the next target to select
        callback_speed(speeds[0][index_target], speeds[1])
        
        pass
