from typing import Callable

class OdomData:

    def __init__(self, x, y, rotation):
    #No Y if speed
        self.x = x
        self.y = y
        self.rotation_rad = rotation
        self.previous_x = x
        self.previous_y = y
        self.previous_rotation_rad = rotation

    def updataOdomData(self, x, y, rotation):
        self.previous_x = self.x
        self.previous_y = self.y
        self.previous_rotation_rad = self.rotation_rad

        self.x = x
        self.y = y
        self.rotation_rad = rotation
        
    def set_nope(self): #set outside the map
        self.x = -1
        self.y = -1

    def is_nope(self): #is outside the map (invalid odomData)
        return self.x <0 or self.y <0 or self.x>3 or self.y > 2

class NavigationType():
    def __init__(self, fixed_obstacles = None):
        pass

    def set_target(self, target_pose:OdomData):
        pass

    def update_dyn_obstacles(self, dynamic_obstacles = None):
        pass
    
    def update_odom(self, callback_speed:Callable[[float, float], None], position:OdomData, speed:OdomData, dt=0.0):
        """_summary_

        Args:
            callback_speed (Function): Function to call with speed as parameter (linear in m/s, angular in rad/s)
            position (_type_): _description_
            speed (_type_): _description_
        """
        pass

    #def reset(self):
        #pass
