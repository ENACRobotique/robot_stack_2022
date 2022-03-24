import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class lidarlocation(Node):
    def __init__(self,name='Lidar_location'):
        super().__init__(name)
        self.subscription= self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_=self.create_publisher(LaserScan,'filtered_scan',10)
        

    def listener_callback(self,msg):
        self.get_logger().info(msg.ranges)
        msg_out = generate_filtered_message(msg,filter_out(msg))
        self.get_logger().info(msg_out.ranges)
        self.publisher_publish(msg_out) 
             
    def filter_out(message):
        out = []
        for i in range(0, len(message['ranges'])):
            if message['ranges'][i] is not None :
                if message['ranges'][i] > 3.6 :
                    out.append(None)
                else:
                    out.append(message['ranges'][i])
            else:
                out.append(message['ranges'][i])
        return out

    def generate_filtered_message(message, filtered_data):
        out = message
        out['ranges'] = filtered_data
        return out

def main():
    rclpy.init()
    node = lidarlocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()