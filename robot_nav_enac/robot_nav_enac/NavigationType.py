class OdomData:

	def __init__(self, x, y, rotation):
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


class NavigationType():
    def __init__(self, fixed_obstacles = None):
        pass

    def set_target(self, target_pose, dynamic_obstacles = None):
        pass

    def update_odom(self, callback_speed, position, speed):
        """_summary_

        Args:
            callback_speed (Function): Function to call with speed as parameter (linear in m/s, angular in rad/s)
            position (_type_): _description_
            speed (_type_): _description_
        """
        pass

    #def reset(self):
        #pass
