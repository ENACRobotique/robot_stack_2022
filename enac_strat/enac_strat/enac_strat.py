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
    init = State('Init', initial=True)
    yellow = State('Yellow')
    red = State('Red')
    
    slowdown = green.to(yellow)
    stop = yellow.to(red)
    go = red.to(green)

class Strategy(Node):
    def __init__(self):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("enac_strat")

        #ros parameter to add raw_serial feed
        raw_serial_param = self.declare_parameter("enable_raw_serial", True)
        self.enable_raw_serial = raw_serial_param.get_parameter_value().bool_value
        if self.enable_raw_serial:
            self.raw_serial_pub = self.create_publisher(String, "raw_serial", 10)

        #paramétrage ROS
        self.ros_raw_serial = self.create_publisher(String, "/raw_serial", 10)
        self.ros_send_serial = self.create_publisher(String, "/send_serial", 10) #msg sent to robot
        self.ros_odom = self.create_publisher(Odometry, '/odom', 10)
        self.ros_peripherals = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        

        self.ros_vel_listener = self.create_subscription(Twist, '/cmd_vel', self.on_ros_cmd_vel, 10)
        self.ros_pid_listener = self.create_subscription(Pid, '/pid', self.on_ros_pid, 10)
        self.ros_periph_listener = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph_cmd, 10)

        self.start_serial_read()


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