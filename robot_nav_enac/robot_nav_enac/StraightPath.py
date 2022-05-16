from math import atan2, cos, sin, pi

from robot_nav_enac.Acceleration import Acceleration
from interfaces_enac.msg import _set_navigation

SetNavigation = _set_navigation.SetNavigation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry


"""
input synthax :
s       stop
a 1     advance for 1 sec
b 1     backward for 1 sec
l 180   turn left 180°
r 180   turn right 180°

"""

"""
Basic nav : Connu (x,y) -> position, unité Mètre + Orientation () [Message ros odometrie]
+ Destination (x,y) -> position, unité Mètre + Orientation () [//]

Recepetion : vitesse + position

Check : vitesse max : anticiper freinage 
Sortie : Tourner, puis ligne droite, orientation

2e niveau : Update de consignes : recommencer

https://pypi.org/project/simple-pid/ --> PID 
"""

"""
Basic navigation script : 
Input : take a destination position + rotation [ROS odometrie classic msg] + Max Speed
Result : Move Robot and stop at the right point, In case of update, stop robot and restart with new data
"""

class OdomData:

	def __init__(self, x, y, rotation):
		self.x = x
		self.y = y
		self.rotation_rad = rotation
		self.previous_x = x
		self.previous_y = y
		self.previous_rotation_rad = rotation


	def __str__(self):
		return f"x: {self.x} y:{self.y} rot(rd):{self.rotation_rad}"

	def set_nope(self):
		self.x = -1
		self.y = -1

	def is_nope(self):
		return self.x <0 or self.y <0 or self.x>3 or self.y > 2

	def updataOdomData(self, x, y , rotation):
		self.previous_x = self.x
		self.previous_y = self.y
		self.previous_rotation_rad = self.rotation_rad

		self.x = x
		self.y = y
		self.rotation_rad = rotation
	
#TODO : ajoute rrampe d'accélération
class StraightPath():
	
	#### Goal Pose : rotation de fin de déplacement??
	
	def __init__(self, logger, maxSpeed=None, wheel_radius=0.885):
		self.logger = logger
		self.wheel_radius = wheel_radius

		self._isNavigating = False
		self._isRotating = False

		self.current_position = OdomData(0.1400, 1.1400, 0)
		self.target = OdomData(-1, -1, -1)
		self.speed = OdomData(0,0,0)
		
		self.rotation_precision = 0.08 #~4.5 deg
		self.position_precision = 0.1 # 10 cm

		self.accel_linear = Acceleration(0.6, 0.1, 2.0, 1.0)
		self.accel_rotat = Acceleration(1.85, 0.05, 2.0, 1.0)
	
	def set_target(self, target_pose:OdomData):
		if self.target != target_pose: #TODO : check if it's enough to avoid "jerking" from acceleration module
			self.target = target_pose
			self.accel_linear.reset_accel()
			self.accel_rotat.reset_accel()
			self.logger("updated target in StraightPath navigationType")
		#TODO : take into account obstacles

	def update_dyn_obstacles(self, dyn_obstacles = None):
		pass
		#TODO : take into account obstacles

	def update_odom(self, callback_speed, position, speed, dt):

		self.current_position = position
		self.speed = speed
		x = position.x
		y = position.y

		is_not_at_target = abs(x - self.target.x) >= self.position_precision or abs(y - self.target.y) >= self.position_precision #check only position not rotation
		
		rotation_to_target = self.angle_to_target(self.target, self.current_position) #calculate rotation between target position and current position
		relative_rotation_rad = self.diff_angle(rotation_to_target, self.current_position.rotation_rad)

		rotation_to_final_angle = self.diff_angle(self.target.rotation_rad, self.current_position.rotation_rad)

		if  abs(relative_rotation_rad) > self.rotation_precision and is_not_at_target: #first rotation
			self.accel_linear.reset_accel()
			rot_speed = self.get_rotate_speed(relative_rotation_rad, speed.rotation_rad,dt)
			self.logger(f"Rotating with relative angle to target of : {relative_rotation_rad} at speed {rot_speed}")
			self._isNavigating = False
			self._isRotating = True
			callback_speed(0, rot_speed)
			return

		elif abs(relative_rotation_rad) <= self.rotation_precision and is_not_at_target : #aligned to path
			self.accel_rotat.reset_accel()
			lin_speed = self.get_linear_speed(speed.x, dt)
			self._isNavigating = True
			self._isRotating = False
			callback_speed(lin_speed, 0)
			self.logger(f"Aligned to path - going forward at {lin_speed}")
			return
			
		elif not (is_not_at_target) and abs(rotation_to_final_angle) > self.rotation_precision: #final alignment
			rot_speed = self.get_rotate_speed(rotation_to_final_angle, speed.rotation_rad, dt)
			self.logger(f"At target - Aligning to angle {rotation_to_final_angle} at speed {rot_speed}")
			self._isNavigating = False
			self._isRotating = True
			callback_speed(0, self.get_rotate_speed(rotation_to_final_angle, speed.rotation_rad, dt))
			
			return
		else:
			#stop
			self.logger("At target - Stopping")
			callback_speed(0,0)
			return

	
	def get_rotate_speed(self, relative_rotation_rad, cur_rot_speed_rad, dt):
		distance = abs(2 * relative_rotation_rad/pi * self.wheel_radius) #TODO : calculate distance to target
		return self.accel_rotat.get_speed(self.speed.rotation_rad, relative_rotation_rad, dt)

		#if (relative_rotation_rad <= 0):
		#	rot_speed = -0.5
		#else:
		#	rot_speed = 0.5
		#if abs(relative_rotation_rad) <= self.rotation_precision:
		#	rot_speed = 0
		#
		#return rot_speed
	
	def get_linear_speed(self, cur_lin_speed, dt):
		distance = ((self.current_position.x - self.target.x)**2 + (self.current_position.y - self.target.y)**2 )**0.5
		return self.accel_linear.get_speed(cur_lin_speed, distance, dt)
		#TODO : speed curve depending on distance
		#speed = 0.5 #m/s

		#print("Move: vlin: "+str(speed))

		#if distance < 0.1 : 
		#	#To Test
		#	return float(0)
		#else:
		#	return float(speed)

	def diff_angle(self, target, current):
		#https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
		return atan2(sin(target-current), cos(target-current))

	def angle_to_target(self, target:OdomData, current:OdomData):
		#https://blog.finxter.com/calculating-the-angle-clockwise-between-2-points/
		r = atan2(target.y - current.y, target.x - current.x)
		print("Angle to target: "+str(r))
		#r = (v2_theta - v1_theta)
		if r < 0:
			r % pi
		return r

if __name__ == '__main__':
	pass




