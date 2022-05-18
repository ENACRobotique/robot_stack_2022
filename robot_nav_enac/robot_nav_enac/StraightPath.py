from math import atan2, cos, sin, pi, copysign

from robot_nav_enac.Acceleration import Acceleration
from interfaces_enac.msg import _set_navigation
from robot_nav_enac.NavigationType import OdomData

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

How does it work :
1. Align to target
2. go in straight path (correcting for deviation in path by constantly checking angle to target, due to the not perfect angle)
3.
"""
#TODO : ajoute rrampe d'accélération
class StraightPath():
	
	#### Goal Pose : rotation de fin de déplacement??
	
	def __init__(self, logger, maxSpeed=None, wheel_radius=0.885):
		self.logger = logger
		self.wheel_radius = wheel_radius

		self._isNavigating = False
		self._isRotating = False
		self.nav_state = -1
		#nav state :
		#0 : first rotation with ramp
		#1 : first rotation without ramp
		#2 : straight line with ramp
		#3 : straight line without ramp
		#4 : last rotation with ramp
		#5 : last rotation without ramp

		self.current_position = OdomData(0.1400, 1.1400, 0)
		self.target = OdomData(-1, -1, -1)
		self.speed = OdomData(0,0,0)
		
		self.min_rotation_precision = 0.04 #~4.5 deg
		self.max_rotation_precision = 0.20 #~ 13 deg
		#Explanation -> When travelling in straight line, if the robot stop before reaching the perfect angle, there'll be a time during a travel when it has to "correct for the angle"
		#so we need to set a value when the robot can start moving forward (under max_rotation_precision) and stop moving forward if too far from the target (above max_rotation_precision), and stop moving for "precision rotation" (under min_rotation_precision)
		self.position_precision = 0.02 # 2 cm

		self.corr_lin_speed = 0.05 #m/s
		self.corr_ang_speed = 0.05 #rad/s
		self.accel_linear = Acceleration(0.6, 0.1, 3.0, 2.0)
		self.accel_rotat = Acceleration(1.2, 0.05, 2.0, 2.0)
	
	def set_target(self, target_pose:OdomData):
		if self.target != target_pose: #TODO : check if it's enough to avoid "jerking" from acceleration module
			self.target = target_pose #TODO : check out of bounds
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
		if self.target.is_nope(): #prevent moving if no cons was sent
			return

		#if on first rotation (triggered only when "above max_rotation_precision" when travelling or above min_rotation_precision if not at target)
		if  ((abs(relative_rotation_rad) > self.max_rotation_precision and is_not_at_target) 
			or (not self._isNavigating and abs(relative_rotation_rad) > self.min_rotation_precision)): 
			self._isNavigating = False
			if self.nav_state > 1 or self.nav_state < 0: #not in first rotation state
				self.nav_state = 0
				self.accel_rotat.reset_accel()
			if self.nav_state == 0: #if in ramp state
				rot_speed = self.get_rotate_speed(relative_rotation_rad,dt)
				if rot_speed >= -self.corr_ang_speed and rot_speed <= self.corr_ang_speed : #really slow rot_speed
					self.nav_state = 1
			if self.nav_state == 1: #if ramp is over and still not at target
				rot_speed = self.get_rotate_correction_speed(relative_rotation_rad)
				self.logger("robot is using minimal speed due to acceleration ramp finished")

			self.logger(f"Rotating with relative angle to target of : {relative_rotation_rad}  at speed {rot_speed}")
			self._isNavigating = False
			self._isRotating = True
			callback_speed(0, rot_speed)
			return

		elif abs(relative_rotation_rad) <= self.max_rotation_precision and is_not_at_target : #aligned to path
			if self.nav_state < 3 or self.nav_state > 4:
				self.nav_state = 3
				self.accel_linear.reset_accel()
			if self.nav_state == 3:
				lin_speed = self.get_linear_speed(speed.x, dt)
			#TODO : check if need to code state 4

			self._isNavigating = True 
			#if the robot is deviating from the straight line due to angle errors
			if abs(relative_rotation_rad) >= self.min_rotation_precision:
				self._isRotating = True
				rot_speed = self.get_rotate_correction_speed(relative_rotation_rad)
				self.logger(f"rotating at speed {rot_speed} due to having an angle error above {self.min_rotation_precision}")
			else:
				self._isRotating = False
				rot_speed = 0

			callback_speed(lin_speed, rot_speed)
			self.logger(f"Aligned to path - going forward at {lin_speed}")
			return
			
		elif not (is_not_at_target) and abs(rotation_to_final_angle) > self.min_rotation_precision: #final alignment
			if self.nav_state < 5 or self.nav_state > 6:
				self.nav_state = 5
				self.accel_linear.reset_accel()
			if self.nav_state == 5:
				rot_speed = self.get_rotate_speed(rotation_to_final_angle, dt)
			#TODO : check if need to implement state 6
			self.logger(f"At target - turning with speed {rot_speed}")
			self._isNavigating = False
			self._isRotating = True
			callback_speed(0, rot_speed)
			
			return
		else:
			#stop
			self.logger("At target - Stopping")
			callback_speed(0,0)
			return

	def get_rotate_speed(self, relative_rotation_rad, dt):
		sign = lambda x: copysign(1, x) #https://moonbooks.org/Articles/How-to-get-the-sign-of-a-number-eg--1-or-1-in-python-/
		distance = abs(2 * relative_rotation_rad/pi * self.wheel_radius) #TODO : calculate distance to target
		direction = sign(relative_rotation_rad) #Acceleration don't manage negative values
		return direction * self.accel_rotat.get_speed(abs(self.speed.rotation_rad), distance, dt)


	def get_rotate_correction_speed(self, relative_rotation_rad): #quick correction at "small speed" without ramp to correct when traveling in straight line if not following correctly
		if (relative_rotation_rad <= 0):
			rot_speed = -1 #take ~ 2 second to go from min_precision to max_precision
		else:
			rot_speed = 1
		return rot_speed

	def get_linear_speed(self, cur_lin_speed, dt):
		distance = ((self.current_position.x - self.target.x)**2 + (self.current_position.y - self.target.y)**2 )**0.5
		return self.accel_linear.get_speed(cur_lin_speed, distance, dt)

	def get_linear_correction_speed(self): #used if the robot has overshooted or undershooted the target after the accel/deceleration ramp
		distance = ((self.current_position.x - self.target.x)**2 + (self.current_position.y - self.target.y)**2 )**0.5
		speed = 0.1 #m/s
		return float(speed) if distance > 0.1 else float(0)

	def diff_angle(self, target, current):
		#https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
		return atan2(sin(target-current), cos(target-current))

	def angle_to_target(self, target:OdomData, current:OdomData):
		#https://blog.finxter.com/calculating-the-angle-clockwise-between-2-points/
		r = atan2(target.y - current.y, target.x - current.x)
		#r = (v2_theta - v1_theta)
		if r < 0:
			r % pi
		return r

if __name__ == '__main__':
	pass




