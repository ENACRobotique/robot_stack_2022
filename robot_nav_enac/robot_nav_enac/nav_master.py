import numpy as np

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from interfaces_enac.msg import _set_navigation
from interfaces_enac.msg import _obstacles
from interfaces_enac.msg import _periph_value

from robot_nav_enac.Stop import Stop
from robot_nav_enac.StraightPath import StraightPath
from robot_nav_enac.PurePursuit import PurePursuit
from robot_nav_enac.WallStop import WallStop
from robot_nav_enac.WallFollower import WallFollower
from robot_nav_enac.NavigationType import OdomData

SetNavigation = _set_navigation.SetNavigation
Obstacles = _obstacles.Obstacles
PeriphValue = _periph_value.PeriphValue

"""
Navtypes :
    0 : stop
    1 : StraigthPath (without planification, no basic obstacle stop yet)
    2 : PurePursuit (use astar planification and obstacle avoidance)
    3 : WallFollower (without planification, no basic obstacle stop yet)
    4 : WallStop (without planification, no basic obstacle stop yet)
    5 : WallStopBackward
"""

def z_euler_from_quaternions(qx, qy, qz, qw):
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(t3, t4)

# angles en radians
def pol_to_cart(x_rob, y_rob, theta_rob, offset_lidar, angle_loc_obst, dist_obst):
    offset_obst_x = dist_obst * math.cos(angle_loc_obst - offset_lidar + theta_rob)
    offset_obst_y = dist_obst * math.sin(angle_loc_obst - offset_lidar + theta_rob)
    return (x_rob + offset_obst_x, y_rob + offset_obst_y)

def is_obstacle_on_table(distance_from_robot):
    #suppose que l'obstacle est parfaitement devant le robot
    
    pass
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.last_time_stamp = -1.0
        self.dt = 0.0

        self.is_stopped = False
        self.last_nav_type = 0 #to resume navigation after obstacle
        #epaisseur roue : 0.022
        #distance max entre roues (de bout à bout ) : 0.199
        #diameter = 0.177
        
        self.target_position = OdomData(0, 0, 0) #TODO : set it to initial position from state machine on beggining
        self.cur_position = OdomData(0, 0, 0)
        self.cur_position_wheel = OdomData(0,0,0)
        self.cur_speed = OdomData(0,0,0)
        self.cur_speed_wheel = OdomData(0,0,0)
        self.fixed_obstacle = [] #TODO : to fill on init
        self.dynamic_obstacle = [] 

        self.capteur_1 = False
        self.capteur_2 = False

        #instantiate nav types available
        self.stop = Stop()
        self.straight_path = StraightPath(self.get_logger().info)
        self.pure_pursuit = PurePursuit()
        self.wall_follower = WallFollower(self.get_logger().info, lambda: self.capteur_1, lambda: self.capteur_2, 0.2)
        self.wall_stop = WallStop(self.get_logger().info, lambda: self.cur_speed_wheel.x, 0.2)
        self.wall_stop_backward = WallStop(self.get_logger().info, lambda: self.cur_speed_wheel.x, -0.2)

        self.navigation_type = self.stop #default navigation_type by safety
        self.nav_type_int = 0 #used to detect if navigation_type has changed


        # subscribe to nav
        navigation_subscriber = self.create_subscription(
            SetNavigation, 'navigation', self.on_nav_callback, 10)
        #subscribe to odom
        odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.on_odom_callback, 10)
        odom_wheel_subscriber = self.create_subscription( 
                Odometry, 'odom_wheel', self.on_odom_wheel_callback, 10)
        #subscribe to peripherals
        peripheral_sub = self.create_subscription( 
                PeriphValue, 'peripherals', self.on_ros_periph_cmd, 10)
        #subscribe to obstacles
        obstacle_subscriber = self.create_subscription(
            Obstacles, 'obstacles', self.on_obstacle_callback, 10)
        proximity_warning_sbuscriber = self.create_subscription(
            Float32, 'front_distance', self.on_front_distance, 10)

        
        #publish to velocity
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def on_obstacle_callback(self, msg):
        #TODO : maintain a list of dynamic obstacles and send it to PurePursuit astar planification on callback
        pass

    def on_front_distance(self, msg):
        if msg.data <= 0.5 and not self.is_stopped: #object in front at less than 0.5m
            self.last_nav_type = self.nav_type_int
            self.nav_type_int = 0
            self.navigation_type = self.stop
            self.is_stopped = True
            self.get_logger().warn(f"stopping nav due to obstacle at {0.5} m ")
        if msg.data >= 0.5 and self.is_stopped: #resume navigation after proximity obstacle is leaving the vincinity of the robot
            self.is_stopped = False
            self.nav_type_int = self.last_nav_type
            self.assign_navigation_from_int()
            self.navigation_type.set_target(self.target_position) #restart the ramp for straightPath
            self.get_logger().info("resuming nav")

    def on_nav_callback(self, msg):
        #switch nav type if changed
        if self.nav_type_int != msg.navigation_type:
            self.nav_type_int = msg.navigation_type
            self.assign_navigation_from_int()

        #extracting goal_pose from msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        rotation = z_euler_from_quaternions(msg.pose.orientation.x,
        									msg.pose.orientation.y,
        									msg.pose.orientation.z,
        									msg.pose.orientation.w)

        #set target to the navigation_type selected
        self.target_position.updataOdomData(x,y, rotation)
        self.navigation_type.set_target(self.target_position)

        self.get_logger().info(f"receiving nav_cons to target ({x}, {y}) and angle {rotation}")


    def on_odom_wheel_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rotation = z_euler_from_quaternions(msg.pose.pose.orientation.x,
        									msg.pose.pose.orientation.y,
        									msg.pose.pose.orientation.z,
        									msg.pose.pose.orientation.w)

        self.cur_position_wheel.updataOdomData(x, y, rotation)
        self.cur_speed_wheel.updataOdomData(msg.twist.twist.linear.x, 0, msg.twist.twist.angular.z)


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
        #print("Le robot est à (x {},y {},r {})".format(x, y, rotation))
        
        self.cur_position.updataOdomData(x, y, rotation)
        self.cur_speed.updataOdomData(msg.twist.twist.linear.x, 0, msg.twist.twist.angular.z)

        dt = 0.0
        if self.last_time_stamp == -1.0:
            self.last_time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = 0.05 #TODO : not zero in case it could create problem, need to check that
        else:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            dt = timestamp - self.last_time_stamp
            self.last_time_stamp = timestamp
            
        self.navigation_type.update_odom(self.publish_nav, self.cur_position, self.cur_speed, dt) #TODO voir quel type de données mettre (OdomData ??)
        

    def on_ros_periph_cmd(self, msg):
        if msg.header.frame_id == "stm32":#to prevent looping from self messages
            id = msg.periph_name[:2]
            if id == 'c1':
                self.capteur_1 = True if msg.value == 1 else False
                #self.get_logger().info(f'cpt1 : {self.capteur_1} cpt_2 {self.capteur_2}')
            if id == 'c2':
                self.capteur_2 = True if msg.value == 1 else False
                #self.get_logger().info(f'cpt1 : {self.capteur_1} cpt_2 {self.capteur_2}')

    def publish_nav(self, linear_speed: float, angular_speed: float):
        msg = Twist()
        msg.linear.x = float(linear_speed)
        msg.angular.z = float(angular_speed)
        self.velocity_publisher.publish(msg)

    def assign_navigation_from_int(self):
        nb = self.nav_type_int
        if nb == 0:
                self.navigation_type = self.stop
        elif nb == 1:
            self.navigation_type = self.straight_path
            #self.navigation_type.set_target = cur_position or reset button??
        elif nb == 2:
            self.navigation_type = self.pure_pursuit
        elif nb == 3:
            self.navigation_type = self.wall_follower
        elif nb == 4:
            self.navigation_type = self.wall_stop
        elif nb == 5:
            self.navigation_type = self.wall_stop_backward
        else:
            self.get_logger().error('wrong navigation type sent : not between 0 and 5 included ')
	


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

