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

    def generate_filtered_message(self,message, filtered_data):
        out = message
        out.ranges = filtered_data
        return out    
    
    def filter_out(self,message):
        out = []
        for i in range(0, len(message.ranges)):
            if message.ranges[i] is not None :
                if message.ranges[i] > 3.6 :
                    out.append(None)
                else:
                    out.append(message.ranges[i])
            else:
                out.append(message.ranges[i])
        return out

    def echo(self, messsage):
        return message

    def listener_callback(self,msg):
        #self.get_logger().info(msg.angle_max)
        msg_out = self.generate_filtered_message(msg,self.filter_out(msg))
        #msg_out = msg
        #self.get_logger().info(msg_out.angle_max)
        self.publisher_.publish(msg_out) 
 
    

    

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