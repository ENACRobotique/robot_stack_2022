import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry

""" 
Navtypes :
    0 : stop
    1 : StraigthPath (without planification, no basic obstacle stop yet)
    2 : PurePursuit (use astar planification and obstacle avoidance)
    3 : WallFollower (without planification, no basic obstacle stop yet)
    4 : WallStop (without planification, no basic obstacle stop yet)
"""

class OdomData:

	def __init__(self, x, y, rotation):
		self.x = x
		self.y = y
		self.rotation_rad = rotation
		self.previous_x = x
		self.previous_y = y
		self.previous_rotation_rad = rotation


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

		

		#Stop point to be computed, to know when starting to stop (in regards of PID)
		#self.stopPoint = OdomData(0.0, 0.0, 360)

	
		#self.stopPoint = stopLoc() #Thread it after successful tests	
	
	def setTarget(self, msg):
		#fetch goal_pose message and update target pose
		x = msg.position.x
		y = msg.position.y
		rotation = z_euler_from_quaternions(msg.orientation.x,
											msg.orientation.y,
											msg.orientation.z,
											msg.orientation.w)
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

		if rotation - self.target.rotation_rad > self.rotationationPrecision : #meter
			if speed >= 0.01 :
				self.stop()
				return
			
			self._isNavigating = False
			self._isRotating = True
			#Need rotationation
			self.rotate()
			return

		elif x - self.target.x <= self.position_precision and y - self.target.y <= self.position_precision :
			self._isNavigating = True
			self._isRotating = False
			self.move()
			return

		self._isNavigating = False
		self._isRotating = False
		
	def stop(self):
		msg = Twist()
		#publish empty message to velocity to stop robot
		self.velocity_publisher.publish(msg)
	
	def rotate(self):
		relative_rotation_rad = self.current_position.rotation_rad  - self.target.rotation_rad

		if (relative_rotation_rad <= 0):
			speed = -2
		else:
			speed = 2
		
		msg = Twist()

		msg.twist.linear.x = 0.0
		msg.twist.linear.y = 0.0
		msg.twist.linear.z = 0.0
		msg.twist.angular.x = 0.0
		msg.twist.angular.y = 0.0
		msg.twist.angular.z = float(speed)

		self.velocity_publisher.publish(msg)
	
	def move(self):
		distance = ( (self.current_position.x - self.target.x)**2 + (self.current_position.y - self.target.y)**2 )**0.5

		#TODO : speed curve depending on distance
		speed = 0.5 #m/s

		if distance <= 0.1 : 
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

