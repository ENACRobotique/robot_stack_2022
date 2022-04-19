import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from interfaces_enac import __sensorvalue, __pid

PeriphValue = __periphvalue.PeriphValue
Pid = __pid.Pid

import serial
import threading

CMD_VEL = "v {} {}"
CMD_STOP = "s"
CMD_ACTU = "a {} {}"
CMD_DECL = "d"
CMD_PID = "g {} {} {}"

MESSAGE = "m"
ODOM_MOTOR = "p"
PERIPH_DECL = "b"
CAPT_VAL = "c"

#commandes à envoyer au serial:
#v <int> <int>: commande de vitesse <linéaire * 1000> <omega * 1000>
#s: arrêt du robot (ignoré: RoboKontrol)
#a <id> <int> : ordre à un actionneur.
#   id est deux caractères (les deux premières lettres de son nom a6_MAIN_AVANT, donc a6), le premier donnant le type d'actionneur (a pour AX12A, p pour pompe, e pour electroVanne, s pour servo)
#   le deuxième est un chiffre d'identification.
#d : demande de description des actionneurs (ignoré: RoboKontrol)
#g [o/v] <int> <int> : changement des valeurs des PIDs (kp et ki)

#commandes à récupérer du serial:
#m <string>
#p <double> <double> <double> <double> <double>: odométrie moteur <x> <y> <théta> <vlin> <vtheta>
#b <string> <int> <int> <int> [R/RW] <string>: déclaration d'un actionneur (RW) ou d'un capteur (R). (ignoré: RoboKontrol)
#c <string> <int> : retour de capteur

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class Ros2Serial(Node):
    def __init__(self, port, timeout = 0.05, baudrate = 115200, rx_buffer_size=64, tx_buffer_size=64):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("ros2serial")
        #paramétrage serial
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)
        self.thread_read = threading.Thread(target=self.serial_read)
        self.listen = True

        #ros parameter to add raw_serial feed
        raw_serial_param = self.declare_parameter("enable_raw_serial", True)
        self.enable_raw_serial = raw_serial_param.get_parameter_value().value
        if self.enable_raw_serial.get_parameter_value().value:
            self.raw_serial_pub = self.create_publisher(String, "raw_serial", 10)

        #paramétrage ROS
        self.ros_odom = self.create_publisher(Odometry, '/odom', 10)
        self.ros_peripherals = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        self.ros_vel_listener = self.create_subscription(Twist, '/cmd_vel', self.on_ros_cmd_vel, 10)
        self.ros_vel_listener = self.create_subscription(Pid, '/pid', self.on_ros_pid, 10)
        self.ros_periph_listener = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph_cmd, 10)

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
            if len(message) > 2:
                #dispatch message on the right topic
                if message[0] == MESSAGE:
                    self.on_serial_msg(message[2:])
                elif message[0] == ODOM_MOTOR:
                    self.on_serial_odom(message.split(' ')[1:])
                elif message[0] == PERIPH_DECL:
                    self.on_serial_periph(message)
                elif message[0] == CAPT_VAL:
                    self.on_serial_capt(message.split(' ')[1:])
                
                #dispatch message on the raw_serial topic
                if self.enable_raw_serial:
                    self.raw_serial_pub.publish('ser>node  |  ' + message)

    def on_serial_msg(self, arg): #TODO: Tester
        #convertir les infor reçues au format ROS2
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = 0
        reference.name = "ros2serial_message"
        reference.message = arg
        reference.hardware_id = "stm32"
        reference.values = []
        msg.status = [reference]
        self.ros_diagnostics.publish(msg)

    def on_serial_odom(self, args):
        #convertir les infor reçues au format ROS2
        msg = Odometry() #TODO: args
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "robot"
        #PoseWithCovariance
        msg.pose.pose.position.x = float(args[0])
        msg.pose.pose.position.y = float(args[1])
        msg.pose.pose.position.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(0.0, 0.0, float(args[2]))
        msg.pose.pose.quaternion.x = qx
        msg.pose.pose.quaternion.y = qy
        msg.pose.pose.quaternion.z = qz
        msg.pose.pose.quarernion.w = qw
        #msg.pose.covariance = []
        #TwistWithCovariance
        msg.twist.linear.x = float(args[3])
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(args[4])

        #envoyer les infos sur le bon topic
        self.ros_publisher_odom.publish(msg)
    
    def on_serial_periph(self, arg):
        #convertir les infor reçues au format ROS2
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = 0
        reference.name = "ros2serial_periphDecl"
        reference.message = arg
        reference.hardware_id = "stm32"
        reference.values = []
        msg.status = [reference]
        self.ros_diagnostics.publish(msg)

    def on_serial_capt(self, args):
        #convertir les infor reçues au format ROS2
        msg = PeriphValue() #TODO: args
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sensor_name = args[0]
        msg.value = int(args[1])
        #envoyer les infos sur le bon topic
        self.ros_peripherals.publish(msg)


    def serial_send(self, msg):
        """Envoyer un message sur le port série"""
        self.ser.write(msg.encode('utf-8'))
        if self.enable_raw_serial:
            self.raw_serial_pub.publish('node>ser  |  ' + msg)

    def on_ros_cmd_vel(self, msg):
        vlin = msg.linear.x
        vtheta = msg.angular.z
        self.serial_send(CMD_VEL.format(int(vlin*1000), int(vtheta*1000)))

    def on_ros_periph_cmd(self, msg):
        id = msg.periph_name[:2]
        cmd = msg.value
        self.serial_send(CMD_VEL.format(id, cmd))

    def on_ros_pid(self, msg):
        kpv = msg.kpv
        kiv = msg.kiv
        #kdv = msg.kdv
        kpo = msg.kpo
        kio = msg.kio
        #kdo = msg.kdo
        if (kpv != 0 or kiv == 0): # or kdv != 0:
            self.serial_send(CMD_PID.format('v', kpv, kiv))
        if (kpo != 0 or kio == 0): # or kdo != 0:
            self.serial_send(CMD_PID.format('o', kpo, kio))


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