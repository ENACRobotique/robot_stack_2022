from enac_strat.enac_strat.conversions import z_euler_from_quaternions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from interfaces_enac.msg import _periph_value, _pid

PeriphValue = _periph_value.PeriphValue
Pid = _pid.Pid

#uses python-statemachine
from statemachine import StateMachine, State

#see for documentation https://github.com/fgmacedo/python-statemachine

class StratStateMachine(StateMachine):
    init = State('Init', initial=True) #état de préparation pré-tirette
    outhome = State('Out_Home') #état de sortie de la zone de départ
    
    start = init.to(outhome)


class Strategy(Node, StateMachine):

    #états
    init = State('Init', initial=True) #état de préparation pré-tirette
    outhome = State('Out_Home') #état de sortie de la zone de départ
    
    #transitions
    start = init.to(outhome)

    #valeurs stockées
    x = 140 #appuyé sur le rebord
    y = 1140 # l'encodeur noir est sur le bord intérieur de la bande jaune
    theta = 0
    vlin = 0
    vtheta = 0
    periphs = {}

    def __init__(self):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("enac_strat")
        self.state_machine = StratStateMachine(self.on_start)

        #paramétrage ROS
        self.ros_periph_pub = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.ros_vel_pub = self.create_subscription(Twist, '/cmd_vel', 10)
        
        self.ros_odom_sub = self.create_subscription(Odometry, '/odom', self.on_ros_odom, 10)
        self.ros_periph_sub = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph, 10)

        self.start_serial_read()
    
    def on_ros_periph(self, msg):
        if (msg.header.frame_id == "stm32"):#to prevent looping from self messages
            id = msg.periph_name[:2]
            cmd = msg.value
            print("on_ros_periph "+str(id)+" "+str(cmd))
            self.periphs[id] = cmd
            self.check_transitions()

    def on_ros_odom(self, msg):
        if (msg.header.frame_id == "map" and msg.child_frame_id == "robot"):
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.theta = z_euler_from_quaternions(qx, qy, qz, qw)
            self.vlin = msg.twist.twist.linear.x
            self.vtheta = msg.twist.twist.angular.z
            self.check_transitions()

    def check_transitions(self):
        try:
            if self.state_machine.is_init:
                if self.periphs["TI"] == 42:
                    self.state_machine.start
        except Exception as e:
            pass
    
    #fonctions des transitions
    def on_start(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    strat = Strategy()

    rclpy.spin(strat)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    strat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()