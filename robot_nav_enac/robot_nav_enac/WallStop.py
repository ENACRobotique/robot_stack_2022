from robot_nav_enac.NavigationType import OdomData, NavigationType, Callable

DEMI_LONGUEUR_ROBOT = 0.14 #in meter
class WallStop(NavigationType):
    def __init__(self, logger, get_coder_speed, speed = 0.2, fixed_obstacles = None):
        self.epsilon_move_from_coder = 0.003 if speed >= 0 else - 0.003#meters
        self.epsilon_min_speed = 0.1 if speed >= 0 else -0.1
        self.reached_wall = False
        self.started_moving_forward = False
        self.speed = speed
        self.get_coder_speed = get_coder_speed
        self.logger = logger
        pass

    def set_target(self, target_pose:OdomData):
        if target_pose.x != 0.0:
            pass #TODO : send reset x to target_pose to low levl
        if target_pose.y != 0.0:
            pass #TODO : send reset y to target_pose to low levl #@ x y theta
        #if theta

        self.reached_wall = False
        self.started_moving_forward = False

        pass

    def update_dyn_obstacles(self, dynamic_obstacles = None):
        pass
    
    def update_odom(self, callback_speed:Callable[[float, float], None], position:OdomData, speed:OdomData, dt=0.0):
        if speed.x >= self.epsilon_min_speed: # * 2 for margin
            self.started_moving_forward = True #make sure that we trigger the detection of wall code if the robot has started to travel

        if self.reached_wall:
            callback_speed(0.0,0.0)
        elif self.started_moving_forward and not self.reached_wall and self.get_coder_speed() <= self.epsilon_move_from_coder:
            self.logger("reached wall !")
            self.reached_wall = True
                #TODO : reset x or y to low level or THETA
        else:
            callback_speed(self.speed, 0.0)
        