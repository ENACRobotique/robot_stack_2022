from operator import truediv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

import serial
import threading

CMD_VEL = "s {} {}"
ODOMETRY_REG = "o" #to be better defined
ACTUATOR_REG = "a" #to be better defined

class Ros2Serial(Node):
    def __init__(self, port, timeout = 0.05, baudrate = 115200, rx_buffer_size=64, tx_buffer_size=64): # default buffer size like teensy
        super().__init__("ros2serial")
        #paramétrage serial
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)
        self.thread_read = threading.Thread(target=self.serial_read)
        self.listen = True
        #paramétrage ROS
        self.ros_publisher_odom = self.create_publisher(Odometry, 'topic', 10)
        self.ros_publisher_actu = self.create_publisher(String, 'topic', 10)

    def start_serial_read(self):
        self.thread_read.start()

    def stop_serial_read(self):
        self.listen = False

    def serial_read(self):
        """Fonction qui lit en permanence le port série.
        S'exécute dans un thread séparé"""
        while self.listen:
            message = self.ser.readLine()
            message = message.decode()
            if len(message) != 0:
                if message[0] == ODOMETRY_REG[0]:
                    self.on_serial_odom(message.split(' ')[1:])
                if message[0] == ACTUATOR_REG[0]:
                    self.on_serial_actu(message.split(' ')[1:])

    def on_serial_odom(self, args):
        #convertir les infor reçues au format ROS2
        msg = Odometry(...) #TODO: args
        ...
        #envoyer les infos sur le bon topic
        self.ros_publisher_odom.publish(msg)
    
    def on_serial_actu(self, args):
        #convertir les infor reçues au format ROS2
        msg = String(...) #TODO: args
        ...
        #envoyer les infos sur le bon topic
        self.ros_publisher_actu.publish(msg)


    def serial_send(self, msg):
        """Envoyer un message sur le port série"""
        self.ser.write(msg.encode('utf-8'))

    def ros_send_odom(self, args):
        pass

    def ros_send_actu(self, args):
        pass

    def on_ros_cmd_vel(self, args):
        pass

def main(args=None):
    rclpy.init(args=args)

    ros_to_serial = Ros2Serial()

    rclpy.spin(ros_to_serial)

    ros_to_serial.start_serial_read()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_to_serial.destroy_node()
    ros_to_serial.stop_serial_read()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
