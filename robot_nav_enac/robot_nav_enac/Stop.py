from robot_nav_enac.NavigationType import NavigationType


class Stop(NavigationType):
    def __init__(self):
        pass

    def set_target(self, target_pose):
        pass

    def update_odom(self, callback_speed, position, speed, dt=0.0):
        callback_speed(0, 0)
        pass
    