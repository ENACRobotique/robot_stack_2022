from robot_nav_enac.NavigationType import NavigationType


class Stop(NavigationType):
    def __init__(self):
        pass

    def set_target(self, target_pose, obstacles=None):
        pass

    def update_odom(self, callback_speed, position, speed):
        callback_speed(0, 0)
        pass
    