import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

def z_euler_from_quaternions(qx, qy, qz, qw):
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(t3, t4)
class AccelCalibration(Node):
    def __init__(self):
        super().__init__('navigator')

        self.epsilon_cons_speed = 0.002 #m/s
        self.cons_speed = -1.0 #m/s
        self.new_cons = False
        self.reached_high = False
        self.testing_linear= True #if false, testing angular
        self.init_timestamp = 0.0
        #subscribe to odom
        odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.on_odom_callback, 10)

        #subscribe to max_speed to test 
        max_speed_subscriber = self.create_subscription(
            Twist, 'max_speed', self.on_max_speed_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def on_max_speed_callback(self, msg): #works only by setting only either cons_speed or rotation speed
        if msg.linear.x != 0 and msg.angular.z != 0:
            self.new_cons = False
            self.cons_speed = -1.0
            self.rotation_speed = -1.0
            self.get_logger().error("AccelCalibration works only by setting either the linear x or the angular z, but not both !")
            return
        elif msg.linear.x != 0:
            self.cons_speed = msg.linear.x
            self.testing_linear = True
        elif msg.angular.z != 0:
            self.cons_speed = msg.angular.z
            self.testing_linear = False

        self.init_timestamp = 0.0
        self.get_logger().info(f"Starting new acceleration test calibration with max linear speed : {self.cons_speed}")
        self.new_cons = True
        

    def on_odom_callback(self, msg):
        linear_speed = msg.twist.twist.linear.x
        angular_speed = msg.twist.twist.angular.z

        if self.new_cons:
            self.reached_high = False
            if linear_speed >= 0 + self.epsilon_cons_speed: #stop the robot for the test if not at zero speed
                self.velocity_publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0,z=0.0)))
                return
            else:
                self.new_cons = False
                if self.testing_linear:
                    self.velocity_publisher.publish(Twist(linear=Vector3(x=self.cons_speed, y=0.0,z=0.0)))
                else:
                    self.velocity_publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0,z=self.cons_speed)))
                self.init_timestamp = self.get_clock().now().nanoseconds
                return
        if self.init_timestamp != 0.0:
            if not self.reached_high and msg.twist.twist.linear.x >= self.cons_speed - self.epsilon_cons_speed:
                self.get_logger().info(f"Reached max speed : {self.cons_speed} in \
                    {(self.get_clock().now().nanoseconds - self.init_timestamp) * 1e-9} secs")
                self.init_timestamp = self.get_clock().now().nanoseconds 
                self.velocity_publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0,z=0.0)))
                self.reached_high = True
            if self.reached_high and msg.twist.twist.linear.x <= 0 + self.epsilon_cons_speed:
                self.reached_high = False
                self.get_logger().info(f"Reached 0 after deceleration in \
                    {(self.get_clock().now().nanoseconds -self.init_timestamp) * 1e-9 } secs")
                self.cons_speed = -1.0
                self.rot_cons_speed = -1.0




def main():
    rclpy.init()
    node = AccelCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()

