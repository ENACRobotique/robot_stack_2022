from robot_nav_enac.NavigationType import NavigationType

class PurePursuit(NavigationType):
    def __init__(self):
        #generate graph with fixed obstacles
        pass

    def set_target(self, target_pose, obstacles=None):
        #astar with dynamic obstacles in parameters
        #self.path_to_follow = result_from_astar
        pass

    def update_odom(self, callback_speed, position, speed):
        #generate cons_speed from pure pursuit algorithm on self.path_to_follow
        pass