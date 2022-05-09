import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry

from interfaces_enac.msg import _set_navigation
from interfaces_enac.msg import _obstacles

from robot_nav_enac.Stop import Stop
from robot_nav_enac.StraightPath import StraightPath
from robot_nav_enac.PurePursuit import PurePursuit

SetNavigation = _set_navigation.SetNavigation
Obstacles = _obstacles.Obstacles

"""
Navtypes :
    0 : stop
    1 : StraigthPath (without planification, no basic obstacle stop yet)
    2 : PurePursuit (use astar planification and obstacle avoidance)
    3 : WallFollower (without planification, no basic obstacle stop yet)
    4 : WallStop (without planification, no basic obstacle stop yet)
"""


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.stop = Stop()
        self.straight_path = StraightPath()

        self.nav_type = self.stop
        self.nav_type_int = 0


        # subscribe to nav
        navigation_subscriber = self.create_subscription(
            SetNavigation, 'set_nav', self.on_nav_callback, 10)
        #subscribe to odom
        odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.on_odom_callback, 10)
        #subscribe to obstacles
        obstacle_subscriber = self.create_subscription(
            Obstacles, 'obstacles', self.on_obstacle_callback, 10)
        #publish to velocity
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel')

    def on_obstacle_callback(self, msg):
        #TODO : maintain a list of dynamic obstacles and send it to PurePursuit astar planification on callback
        pass

    def on_nav_callback(self, msg):
        if self.nav_type_int != msg.nav_type:
            self.nav_type = msg.nav_type
            if self.nav_type == 0:
                self.nav_type = self.stop()
            elif self.nav_type == 1:
                self.nav_type = self.straight_path
                #self.nav_type.set_target = cur_position or reset button??

    
        x = msg.position.x
        y = msg.position.y
        rotation = z_euler_from_quaternions(msg.orientation.x,
        									msg.orientation.y,
        									msg.orientation.z,
        									msg.orientation.w)
        self.target.updataOdomData(x, y, rotation)

        #self.nav_type.set_target(OdomDataObject)


    def on_odom_callback(self, msg):
		# Update position here
		# TODO : uncomment when TF is working
		# x = msg.transform.translation.x
		# y = msg.transform.translation.y
		# rotation = z_euler_from_quaternions(msg.transform.rotation.x,
		#									msg.transform.rotation.y,
		#									msg.transform.rotation.z,
		#									msg.transform.rotation.w)
		# self.current_position.updataOdomData(x, y, rotation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rotation = z_euler_from_quaternions(msg.pose.pose.orientation.x,
        									msg.pose.pose.orientation.y,
        									msg.pose.pose.orientation.z,
        									msg.pose.pose.orientation.w)
        self.current_position.updataOdomData(x, y, rotation)

        speed = msg.twist.twist.linear.x

        self.nav_type.update_odom(self.publish_nav, msg.pose.pose.position, msg.twist.twist.linear.x) #TODO voir quel type de données mettre (OdomData ??)

    def publish_nav(self, linear_speed, angular_speed):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
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

