import numpy as np

from math import atan2, cos, sin, pi

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

#TODO: RAMPES DE VITESSE


def z_euler_from_quaternions(qx, qy, qz, qw):
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(t3, t4)

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
	

class Navigator(Node):
	
	#### Goal Pose : rotation de fin de déplacement??
	
	def __init__(self, xTarget, yTarget, rotationTarget, maxSpeed):
		super().__init__('navigator')

		self._isNavigating = False
		self._isRotating = False

		self._max_speed = maxSpeed
		self.current_position = OdomData(0.1400, 1.1400, 0)
		self.target = OdomData(-1, -1, -1)

		#self.odom_topic = self.create_subscription(TransformStamped, "/odom", self.updatePosition, 10) #odom fused by robot_localization or fed directly when debug
		#self.wheel_topic = self.create_subscription(Odometry, "/encoder", self.updateSpeed, 10) #used to get speed from encoder
		self.odom_topic = self.create_subscription(Odometry, "/odom", self.updatePosition, 10)
		self.goal_pose_topic = self.create_subscription(SetNavigation, "/navigation", self.setTarget, 10)
		self.velocity_publisher = self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		
		self.rotation_precision = 0.08 #~4.5 deg
		self.position_precision = 0.1

		#Stop point to be computed, to know when starting to stop (in regards of PID)
		#self.stopPoint = OdomData(0.0, 0.0, 360)

	
		#self.stopPoint = stopLoc() #Thread it after successful tests	
	
	def setTarget(self, msg):
		#fetch goal_pose message and update target pose
		x = msg.pose.position.x
		y = msg.pose.position.y
		rotation = z_euler_from_quaternions(msg.pose.orientation.x,
											msg.pose.orientation.y,
											msg.pose.orientation.z,
											msg.pose.orientation.w)
		self.target.updataOdomData(x, y, rotation)


	def updatePosition(self, msg):
		#Update position here
		#TODO : uncomment when TF is working
		#x = msg.transform.translation.x
		#y = msg.transform.translation.y
		#rotation = z_euler_from_quaternions(msg.transform.rotation.x,
		#									msg.transform.rotation.y,
		#									msg.transform.rotation.z,
		#									msg.transform.rotation.w)
		#self.current_position.updataOdomData(x, y, rotation)
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		rotation = z_euler_from_quaternions(msg.pose.pose.orientation.x,
											msg.pose.pose.orientation.y,
											msg.pose.pose.orientation.z,
											msg.pose.pose.orientation.w)
		self.current_position.updataOdomData(x, y, rotation)

		speed = msg.twist.twist.linear.x

		is_not_at_target = abs(x - self.target.x) >= self.position_precision or abs(y - self.target.y) >= self.position_precision #check only position not rotation
		
		rotation_to_target = self.angle_to_target(self.target, self.current_position) #calculate rotation between target position and current position
		relative_rotation_rad = self.diff_angle(rotation_to_target, self.current_position.rotation_rad)


		print(f"Update Position pos:({self.current_position}) speed:{speed} tgt:({self.target})")
		if self.target.is_nope():
			return

		if  abs(relative_rotation_rad) > self.rotation_precision and is_not_at_target: #first rotation
			print(">> First rotation")
			self._isNavigating = False
			self._isRotating = True
			#Need rotation
			self.rotate(relative_rotation_rad, rotation_to_target)
			return

		elif abs(relative_rotation_rad) <= self.rotation_precision and is_not_at_target : #aligned to path
			print(">> Aligned to path")
			self._isNavigating = True
			self._isRotating = False
			self.move()
			return
		elif not (is_not_at_target) and abs(self.diff_angle(self.target.rotation_rad, rotation)) > self.rotation_precision: #final alignment
			print(">> Final alignment")
			self._isNavigating = False
			self._isRotating = True
			relative_rotation_rad = self.diff_angle(self.target.rotation_rad, rotation)
			#Need rotation
			self.rotate(relative_rotation_rad, self.target.rotation_rad)
			return
		else:
			self.stop()
			self.target.set_nope()
			return

		#	if speed >= 0.01 :
		#		self.stop()
		#		return
		#self._isNavigating = False
		#self._isRotating = False
		
	def stop(self):
		print("Stop")
		msg = Twist()
		#publish empty message to velocity to stop robot
		self.velocity_publisher.publish(msg)
	
	def rotate(self, relative_rotation_rad, target):
		print("Rotate: "+str(relative_rotation_rad)+" tgt: "+str(target))

		if (relative_rotation_rad <= 0):
			rot_speed = -0.05
		else:
			rot_speed = 0.05
		if abs(relative_rotation_rad) <= self.rotation_precision:
			rot_speed = 0
		msg = Twist()

		msg.linear.x = 0.0
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = float(rot_speed)

		self.velocity_publisher.publish(msg)
	
	def move(self):
		distance = ((self.current_position.x - self.target.x)**2 + (self.current_position.y - self.target.y)**2 )**0.5

		#TODO : speed curve depending on distance
		speed = 0.5 #m/s

		print("Move: vlin: "+str(speed))

		if distance < 0.1 : 
			#To Test
			self.stop()
		else:
			msg = Twist()

			msg.linear.x = float(speed)
			msg.linear.y = 0.0
			msg.linear.z = 0.0
			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0

			self.velocity_publisher.publish(msg)

	def diff_angle(self, target, current):
		#https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles
		return atan2(sin(target-current), cos(target-current))

	def angle_to_target(self, target:OdomData, current:OdomData):
		#https://blog.finxter.com/calculating-the-angle-clockwise-between-2-points/
		v1_theta = atan2(current.y, current.x)
		v2_theta = atan2(target.y, target.x)
		r = atan2(target.y - current.y, target.x - current.x)
		print("Angle to target: "+str(r))
		#r = (v2_theta - v1_theta)
		if r < 0:
			r % pi
		return r

def main():
    rclpy.init()
    node = Navigator(0,0,0,0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()




