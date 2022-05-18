from robot_nav_enac.NavigationType import OdomData, NavigationType

class WallStop(NavigationType):
    def __init__(self, fixed_obstacles = None):
        self.epsilon_move_from_coder = 0.004 #meters
        self.reached_wall = False
        pass

    def set_target(self, target_pose:OdomData):
        if target_pose.x != 0.0:
            pass #TODO : send reset x to target_pose to low levl
        if target_pose.y != 0.0:
            pass #TODO : send reset y to target_pose to low levl
        #if theta

        self.reached_wall = False

        pass

    def update_dyn_obstacles(self, dynamic_obstacles = None):
        pass
    
    def update_odom(self, callback_speed:Callable[[float, float], None], position:OdomData, speed:OdomData, dt=0.0):
        pass
        if not self.reached_wall:
            if coder_wheel_speed <= self.epsilon_move_from_coder:
                self.reached_wall = True
                #TODO : reset x or y to low level or THETA
            else:
                callback_speed(0.2, 0.0)
        else: 
            callback_speed(0.0, 0.0)
        