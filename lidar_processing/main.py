import rclpy
from rclpy.node import Node
from std_msgs import LaserScan

class lidarlocation(Node):
     def __init__(self,, name='Lidar_location', ):
        super().__init__(name)
     self.subscription= self.create_subscription(
         LaserScan,
         'scan',
         self.listener_callback,
         10)
         self.subscription

     def listener_callback(self,msg):
         self.get_logger().info(msg.ranges)    

def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()