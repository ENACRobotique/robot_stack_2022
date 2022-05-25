from robot_nav_enac.NavigationType import OdomData, NavigationType, Callable

#TODO : atteindre la target
class WallFollower(NavigationType):
    def __init__(self, logger, get_capteur_1, get_capteur_2, speed = 0.2, fixed_obstacles = None):
        self.get_capteur_1 = get_capteur_1
        self.get_capteur_2 = get_capteur_2
        self.speed = speed
        self.logger = logger
        self.target_pose = OdomData(0,0,0)
        self.position_margin = 0.04 #in meter
        self.target_reached = False
        pass

    def set_target(self, target_pose:OdomData):
        self.target_pose = target_pose
        self.target_reached = False
        pass

    def update_dyn_obstacles(self, dynamic_obstacles = None):
        pass
    
    def update_odom(self, callback_speed:Callable[[float, float], None], position:OdomData, speed:OdomData, dt=0.0):
        if self.target_pose.x  < position.x - self.position_margin:
            self.speed = abs(self.speed)
        elif self.target_pose.x  > position.x + self.position_margin:
            self.speed = - abs(self.speed)
        else: 
            self.target_reached = True
            self.speed = 0
        if not self.target_reached:
            self.logger(f'capt 1 {self.get_capteur_1()} ---- capt 2 {self.get_capteur_2()}')
            if self.get_capteur_1() and self.get_capteur_2():
                callback_speed(self.speed, 0.0) # (self.speed, 0.05) Speed to the left to make sure that we are in contact with the wall ?
            if not self.get_capteur_1() and self.get_capteur_2(): #starting to deviate to the right ?
                callback_speed(self.speed, 0.25)