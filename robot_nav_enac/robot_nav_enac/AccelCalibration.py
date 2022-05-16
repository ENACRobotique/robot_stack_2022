import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import float64


class AccelCalibration(Node):
    def __init__(self):
        super().__init__('navigator')

        self.epsilon_cons_speed = 0.002 #m/s
        self.cons_speed = -1.0 #m/s
        self.new_cons = False
        self.reached_high = False
        self.init_timestamp = 0.0
        #subscribe to odom
        odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.on_odom_callback, 10)

        #subscribe to max_speed to test 
        max_speed_subscriber = self.create_subscription(
            float64, 'max_speed', self.on_max_speed_callback, 10)

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel')

    def on_max_speed_callback(self, msg):
        self.velocity_publisher.publish(
            Twist(linear=Vector3(msg.data,0,0)))
        self.cons_speed = msg.data
        self.new_cons = True
        

    def on_odom_callback(self, msg):
        linear_speed = msg.twist.twist.linear.x
        if self.new_cons:
            self.reached_high = False
            if linear_speed >= 0 + self.epsilon_cons_speed:
                self.velocity_publisher.publish(Twist(linear=Vector3(0,0,0)))
                return
            else:
                self.new_cons = False
                self.velocity_publisher.publish(Twist(linear=Vector3(self.cons_speed,0,0)))
                self.init_timestamp = self. #TODO :get current timestamp
                return
        if not self.reached_high and msg.twist.twist.linear.x >= self.cons_speed - self.epsilon_cons_speed:
            self.get_logger.info(f"Reached max speed : {self.cons_speed} in \
                {self.get_current_time().nanoseconds - self.init_timestamp.nanoseconds} secs")
            self.init_timestamp = self. #TODO :get current timestamp
            self.velocity_publisher.publish(Twist(linear=Vector3(0,0,0)))
            self.reached_high = True
        if self.reached_high and msg.twist.twist.linear.x <= 0 + self.epsilon_cons_speed:
            self.reached_high = False
            self.get_logger.info(f"Reached 0 after deceleration in \
                {self.get_current_time().nanoseconds - self.init_timestamp.nanoseconds} secs")




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

