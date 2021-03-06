import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ros2serial.conversions import quaternion_from_euler
from interfaces_enac.msg import _periph_value, _pid

PeriphValue = _periph_value.PeriphValue
Pid = _pid.Pid

import serial
import threading

CMD_VEL = "v {} {}\n\r"
CMD_STOP = "s\n"
CMD_ACTU = "a {} {}\n\r"
CMD_DECL = "d\n\r"
CMD_PID = "g {} {} {}\n\r"

MESSAGE = "m"
ODOM_MOTOR = "r"
ODOM_WHEEL = "f"
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
#i <string> : envoie d'un test d'intégration avec son nom

#commandes à récupérer du serial:
#m <string>
#r <double> <double> <double> <double> <double>: odométrie moteur <x> <y> <théta> <vlin> <vtheta>
#b <string> <int> <int> <int> [R/RW] <string>: déclaration d'un actionneur (RW) ou d'un capteur (R). (ignoré: RoboKontrol)
#c <string> <int> : retour de capteur
#t <string> <string> <bool> : retour de test, avec un éventuel message et un booléen de succès

#Exemples de déclaration : 
"""
c s1 130
c LR 99.00
c a4 607
c a5 349
c a6 678
c a7 349
c e1 0
c e2 0
m Color = v
c mv 1
c mr 1
c hv 0
c hr 0
c sc 0
"""



class Ros2Serial(Node):
    def __init__(self, timeout = 0.05, rx_buffer_size=64, tx_buffer_size=64):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("ros2serial")

        #TODO : reswitch
        #TODO : mettre des msg diagonistics si le serial marche pas
        #self.declare_parameter('serial_port', "/dev/ttyUSB0")
        #self.declare_parameter('baudrate', 57600)
        self.declare_parameter('serial_port', "/dev/ttyACM0")
        self.declare_parameter('baudrate', 115200)

        #paramétrage serial and BLOCK the code until the serial port is available
        self.port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate_name = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ser = self.init_serial(timeout)
        #serial.Serial(port=self.get_parameter('serial_port').get_parameter_value().string_value, baudrate=self.get_parameter('baudrate').get_parameter_value().integer_value, timeout=timeout)

        #self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)
        self.thread_read = threading.Thread(target=self.serial_read)
        self.listen = True

        #ros parameter to add raw_serial feed
        raw_serial_param = self.declare_parameter("enable_raw_serial", True)
        self.enable_raw_serial = raw_serial_param.get_parameter_value().bool_value
        if self.enable_raw_serial:
            self.raw_serial_pub = self.create_publisher(String, "raw_serial", 10)

        #paramétrage ROS
        self.ros_raw_serial = self.create_publisher(String, "/raw_serial", 10)
        self.ros_odom = self.create_publisher(Odometry, '/odom', 10)
        self.ros_odom_wheel = self.create_publisher(Odometry, '/odom_wheel', 10)
        self.ros_peripherals = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        

        self.ros_vel_listener = self.create_subscription(Twist, '/cmd_vel', self.on_ros_cmd_vel, 10)
        self.ros_pid_listener = self.create_subscription(Pid, '/pid', self.on_ros_pid, 10)
        self.ros_periph_listener = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph_cmd, 10)
        self.ros_send_serial = self.create_subscription(String, "/send_serial", self.serial_send, 10) #msg sent to robot

        self.start_serial_read()

    def init_serial(self, timeout = 0.05):
        """
            open serial if available and if not, wait until the serial port is connected
        """
        error_msg_sent = False
        while True:
            try:
                self.ser = serial.Serial(port=self.port_name, baudrate=self.baudrate_name, timeout=timeout)
                self.on_serial_connected()
                self.get_logger().info('Serial port is connected again')
                return self.ser
            except serial.serialutil.SerialException as e:
                if not error_msg_sent:
                    self.get_logger().error("Serial port not available, waiting for it to be connected...")
                    self.on_serial_disconnected()
                    error_msg_sent = True

    def start_serial_read(self):
        self.thread_read.start()

    def stop_serial_read(self):
        self.listen = False

    def serial_read(self):
        """Fonction qui lit en permanence le port série.
        S'exécute dans un thread séparé"""

        self.get_logger().info("serial_read thread launched")
        message = ""
        while self.listen:
            try:
                message = self.ser.readline()
                message = message.decode()
                if len(message) > 2:
                    print("serial_read "+message)
                    if message[0] == MESSAGE:
                        self.on_serial_msg(message[2:])
                    elif message[0] == ODOM_MOTOR:
                        self.on_serial_odom(message.split(' ')[1:], False)
                    elif message[0] == ODOM_WHEEL:
                        self.on_serial_odom(message.split(' ')[1:], True)
                    elif message[0] == PERIPH_DECL:
                        self.on_serial_periph(message)
                    elif message[0] == CAPT_VAL:
                        self.on_serial_capt(message.split(' ')[1:])
                #publish for debug purpose the raw serial message from the stm32 serial port
                #self.get_logger().debug("received_serial: "+message)
                self.ros_raw_serial.publish(String(data=message))
            except serial.serialutil.SerialException as e: #if the serial port is disconnected, try to reconnect
                self.ser.close()
                self.init_serial() #blocks loop until reconnected
            except Exception as e:
                print(str(message)+"\n")
                print(type(e))
                print("\n")

    def on_serial_msg(self, arg): #TODO: Tester
        #convertir les infor reçues au format ROS2
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = DiagnosticStatus.OK
        reference.name = "ros2serial_message"
        reference.message = arg
        reference.hardware_id = "stm32"
        reference.values = []
        msg.status = [reference]
        self.ros_diagnostics.publish(msg)

    def on_serial_odom(self, args, wheel_message=False): #wheel_message change the topic of publisher, False -> odom, True -> odom_wheel
        #convertir les infor reçues au format ROS2
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        #PoseWithCovariance
        msg.pose.pose.position.x = float(args[0])/1000
        msg.pose.pose.position.y = float(args[1])/1000
        msg.pose.pose.position.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(0.0, 0.0, float(args[2]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        #msg.pose.covariance = []
        #TwistWithCovariance
        msg.twist.twist.linear.x = float(args[3])/1000
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = float(args[4]) #receiving rad/s

        #envoyer les infos sur le bon topic
        if wheel_message:
            self.ros_odom_wheel.publish(msg)
        else:
            self.ros_odom.publish(msg)
    
    def on_serial_periph(self, arg):
        #convertir les infor reçues au format ROS2
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = DiagnosticStatus.OK
        reference.name = "ros2serial_periphDecl"
        reference.message = str(arg)
        reference.hardware_id = "stm32"
        reference.values = []
        msg.status = [reference]
        self.ros_diagnostics.publish(msg)

    def on_serial_connected(self):
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = DiagnosticStatus.OK
        reference.name = "ros2serial_connection"
        reference.message = "Serial port connected"
        reference.hardware_id = "raspberry"
        reference.values = []
        msg.status = [reference]
        try:
            self.ros_diagnostics.publish(msg)
        except AttributeError:
            self.get_logger().info('Serial port is connected (no diagnostic msg, race condition to fix later)')

    def on_serial_disconnected(self):
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = DiagnosticStatus.ERROR
        reference.name = "ros2serial_connection"
        reference.message = "Serial port disconnected"
        reference.hardware_id = "raspberry"
        reference.values = []
        msg.status = [reference]
        try:
            self.ros_diagnostics.publish(msg)
        except AttributeError:
            self.get_logger().error('Serial port is DISconnected (no diagnostic msg, race condition to fix later)')

    def on_serial_capt(self, args):
        #convertir les infor reçues au format ROS2
        msg = PeriphValue() #TODO: args
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stm32"
        msg.periph_name = args[0]
        msg.value = int(args[1])
        #envoyer les infos sur le bon topic
        self.ros_peripherals.publish(msg)


    def serial_send(self, msg):
        """Envoyer un message sur le port série"""
        self.get_logger().debug("send_serial: "+msg)
        #self.ros_send_serial.publish(String(data=msg))
        self.ser.write(msg.encode('utf-8'))
        if self.enable_raw_serial:
            self.raw_serial_pub.publish(String(data=f'node>ser  |  {msg}'))

    def on_ros_cmd_vel(self, msg):
        vlin = msg.linear.x
        vtheta = msg.angular.z
        print("on_ros_cmd_vel "+str(vlin)+" "+str(vtheta))
        self.serial_send(CMD_VEL.format(int(vlin*1000), int(vtheta*10))) #sending déci rad/s due to not able to read float

    def on_ros_periph_cmd(self, msg):
        if (msg.header.frame_id != "stm32"):#to prevent looping from self messages
            id = msg.periph_name[:2]
            cmd = msg.value
            print("on_ros_periph_cmd "+str(id)+" "+str(cmd))
            self.serial_send(CMD_ACTU.format(str(id), str(cmd)))

    def on_ros_pid(self, msg):
        kpv = msg.kpv
        kiv = msg.kiv
        #kdv = msg.kdv
        kpo = msg.kpo
        kio = msg.kio
        #kdo = msg.kdo
        print("on_ros_pid i:"+str(kpv)+" "+str(kiv)+" o:"+str(kpo)+" "+str(kio))
        if (kpv != 0 or kiv == 0): # or kdv != 0:
            self.serial_send(CMD_PID.format('v', str(kpv), str(kiv)))
        if (kpo != 0 or kio == 0): # or kdo != 0:
            self.serial_send(CMD_PID.format('o', str(kpo), str(kio)))


def main(args=None):
    rclpy.init(args=args)

    ros_to_serial = Ros2Serial()

    rclpy.spin(ros_to_serial)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_to_serial.destroy_node()
    ros_to_serial.stop_serial_read()
    rclpy.shutdown()

if __name__ == '__main__':
    main()