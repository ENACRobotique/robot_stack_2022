import math, time
from enac_strat.conversions import z_euler_from_quaternions, quaternion_from_euler
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from interfaces_enac.msg import _periph_value, _pid, _set_navigation

PeriphValue = _periph_value.PeriphValue
Pid = _pid.Pid
SetNavigation = _set_navigation.SetNavigation

#uses python-statemachine
from statemachine import StateMachine, State

#see for documentation https://github.com/fgmacedo/python-statemachine

class StratStateMachine(StateMachine):
    #états
    init = State("Init", initial=True)
    outhome = State("OutHome")
    chope_palet_un = State("Chope palet un")

    almost_end = State("Almost End")
    end = State("End")

    #transitions
    start = init.to(outhome)
    turn_palet = outhome.to(chope_palet_un)
    a_chope = chope_palet_un.to(almost_end)


    last_ten_seconds = init.to(almost_end) | outhome.to(almost_end) | chope_palet_un.to(almost_end)
    stop = almost_end.to(end)

class Strategy(Node):

    #valeurs stockées
    x = 0.140 #appuyé sur le rebord
    y = 1.140 # l'encodeur noir est sur le bord intérieur de la bande jaune
    theta = 0
    vlin = 0
    vtheta = 0
    periphs = {}

    goalx = 0
    goaly = 0
    goaltheta = 0

    chrono = 0
    end = 100

    def __init__(self):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("enac_strat")

        self.state_machine = StratStateMachine()

        #paramétrage ROS
        self.ros_periph_pub = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.ros_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_pub = self.create_publisher(SetNavigation, '/navigation', 10)
        self.publisher_map = self.create_publisher(
            TransformStamped, 'carte_coins', 10)
        
        self.ros_odom_sub = self.create_subscription(Odometry, '/odom', self.on_ros_odom, 10)
        self.ros_periph_sub = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph, 10)
        print(self)
        self.send_all_diags()

    def send_tf_map_corners(self):
        msg_out_b = TransformStamped()
        msg_out_b.header.frame_id = "map"
        msg_out_b.child_frame_id = "map_0_2000"
        msg_out_b.transform.translation.x = 0.0
        msg_out_b.transform.translation.y = 2.0
        msg_out_b.transform.translation.z = 0.0
        [qxb, qyb, qzb, qwb] = quaternion_from_euler(
            0, 0, 0)

        msg_out_b.transform.rotation.x = qxb
        msg_out_b.transform.rotation.y = qyb
        msg_out_b.transform.rotation.z = qzb
        msg_out_b.transform.rotation.w = qwb
        self.publisher_map.publish(msg_out_b)

        msg_out_c = TransformStamped()
        msg_out_c.header.frame_id = "map"
        msg_out_c.child_frame_id = "map_3000_0"
        msg_out_c.transform.translation.x = 3.0
        msg_out_c.transform.translation.y = 0.0
        msg_out_c.transform.translation.z = 0.0
        [qxc, qyc, qzc, qwc] = quaternion_from_euler(
            0, 0, 0)

        msg_out_c.transform.rotation.x = qxc
        msg_out_c.transform.rotation.y = qyc
        msg_out_c.transform.rotation.z = qzc
        msg_out_c.transform.rotation.w = qwc
        self.publisher_map.publish(msg_out_c)

        msg_out = TransformStamped()
        msg_out.header.frame_id = "map_3000_0"
        msg_out.child_frame_id = "map_3000_2000"
        msg_out.transform.translation.x = 0.0
        msg_out.transform.translation.y = 2.0
        msg_out.transform.translation.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(
            0, 0, 0)

        msg_out.transform.rotation.x = qx
        msg_out.transform.rotation.y = qy
        msg_out.transform.rotation.z = qz
        msg_out.transform.rotation.w = qw
        self.publisher_map.publish(msg_out)

        msg_out_d = TransformStamped()
        msg_out_d.header.frame_id = "map_0_2000"
        msg_out_d.child_frame_id = "map_300_2000"
        msg_out_d.transform.translation.x = 3.0
        msg_out_d.transform.translation.y = 0.0
        msg_out_d.transform.translation.z = 0.0
        [qxd, qyd, qzd, qwd] = quaternion_from_euler(
            0, 0, 0)

        msg_out_d.transform.rotation.x = qxd
        msg_out_d.transform.rotation.y = qyd
        msg_out_d.transform.rotation.z = qzd
        msg_out_d.transform.rotation.w = qwd
        self.publisher_map.publish(msg_out_d)
    
    def __str__(self):
        return "Strategy: state: "+str(self.state_machine.current_state)+" x: "+str(self.x)+" y: "+str(self.y)+" theta: "+str(self.theta)+(" match unstarted "if self.chrono == 0 else " chrono: "+str(time.time() - self.chrono))
    
    def send_all_diags(self):
        if self.chrono == 0:
            self.send_diagnostic(DiagnosticStatus.OK, "Strategy: match", "Match has not started")
        elif self.chrono != 0 and (time.time() - self.chrono) > self.end:
            self.send_diagnostic(DiagnosticStatus.ERROR, "Strategy: match", "Match has ended, strategy is blocked")
        else:
            self.send_diagnostic(DiagnosticStatus.OK if ((time.time() - self.chrono) < self.end - 10) else DiagnosticStatus.WARN, "Strategy: match", f"{str(time.time() - self.chrono)[:7]} seconds have passed")
        self.send_diagnostic(DiagnosticStatus.STALE, "Strategy: state", f"{str(self.state_machine.current_state)}")

        self.send_tf_map_corners()

    def on_ros_periph(self, msg):
        if (msg.header.frame_id == "stm32"):#to prevent looping from self messages
            print("Periph from stm32")
            id = msg.periph_name[:2]
            cmd = int(msg.value)
            print("on_ros_periph "+str(id)+" "+str(cmd))
            self.periphs[id] = cmd
            self.check_transitions()

    def on_ros_odom(self, msg):
        if (msg.header.frame_id == "map" and msg.child_frame_id == "odom"):
            print("Odometry map->odom")
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

    def is_at_goal(self, vlin_tol, vtheta_tol, distmax_mm):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - self.goalx)**2 + (self.y - self.goaly)**2) <= distmax_mm) #TODO: add condition sur theta si utile

    def check_transitions(self):
        print("/// Checking transitions")
        self.send_all_diags()
        try:
            #trucs liés au temps
            if not(self.state_machine.is_almost_end) and self.chrono != 0 and (time.time() - self.chrono) > self.end - 10 and (time.time() - self.chrono) < self.end:
                #go home
                print("Strategy: il reste 10 secondes: retour à la maison")
                self.state_machine.last_ten_seconds()
                #code de trucs à faire quand c'est presque la fin ici
            if self.chrono != 0 and (time.time() - self.chrono) > self.end:
                print("Strategy: Time is up, blocking node on standby")
                self.state_machine.stop()
                #code de trucs à faire quand c'est la fin ici
                self.send_all_diags()
                while (True):
                    time.sleep(1)

            #transitions normales
            if self.state_machine.is_init:
                if self.periphs.get("TI") == 42:
                    self.state_machine.start()
                    self.on_start()
            if self.state_machine.is_outhome:
                if self.is_at_goal(0.0001, 0.0001, 10):
                    self.state_machine.turn_palet() #TODO: add transition and states
                    self.on_turn_palet()
            if self.state_machine.is_chope_palet_un:
                if self.periphs.get("hv") == 1: #il y a un palet dans la main avant
                    self.state_machine.a_chope()
                    self.on_almost_end()
            
    
        except Exception as e:
            print("Strategy: crap in transition")
            print(e)
        
        finally:
            print(self)
    
    def send_nav_msg(self, nav_type, x, y, theta):
        self.goalx = float(x)
        self.goaly = float(y)
        self.goaltheta = float(theta)
        msg = SetNavigation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.navigation_type = int(nav_type)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(0.0, 0.0, float(theta))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.nav_pub.publish(msg)
    
    def send_periph_msg(self, id, cmd):
        msg = PeriphValue()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "raspberry"
        msg.periph_name = str(id)
        msg.value = int(cmd)
        #envoyer les infos sur le bon topic
        self.ros_periph_pub.publish(msg)

    def send_cmd_vel(self, vlin, vtheta):
        msg = Twist()
        msg.linear.x = float(vlin)
        msg.angular.z = float(vtheta)
        self.ros_vel_pub.publish(msg)

    def send_diagnostic(self, level, ref_name, message):
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = level
        reference.name = ref_name
        reference.message = message
        reference.hardware_id = "raspberry"
        reference.values = []
        msg.status = [reference]
        self.ros_diag_pub.publish(msg)

    #fonctions des transitions
    def on_start(self):
        print("Strategy: Tirette détectée: start")
        self.chrono = time.time()
        self.send_nav_msg(1.0, 0.700, 1.140, 0.0)

    def on_turn_palet(self):
        print("Strategy: Arrivé destination: Tourner palet")
        self.send_cmd_vel(0.0, -1.0) #je tente des trucs, faire tourner le robot vers la gauche?
        self.send_periph_msg("ma", 0) #choper un palet au sol

    def on_almost_end(self):
        print("Strategy: returning to home")
        self.send_nav_msg(1, 0.200, 1.200, 0) # retourner au bercail
    
    def on_end(self):
        print("End: stop everything")
        self.send_nav_msg(1, -1, -1, -1) #nav shut up pls
        self.send_cmd_vel(0, 0) # stop

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