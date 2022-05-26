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

from enac_strat.statemachine import State, Transition, StateMachine

#### fin partie générée automatiquement
class Strategy(Node):

    #valeurs stockées
    #TODO: refaire pour gestion violet
    x = 0.140 #appuyé sur le rebord
    y = 1.140 # l'encodeur noir est sur le bord intérieur de la bande jaune
    theta = 0
    vlin = 0
    vtheta = 0
    periphs = {}

    goalx = 0
    goaly = 0
    goaltheta = 0

    score = 4 #pour avoir déposé une statuette et une vitrine sur le terrain
    what_done = "has_stat has_vitr "#indiquer ce qui a été fait
    chrono = 0
    end = 100

    dont_do_shit_anymore = False

    prio_galerie = False

    #vars galerie

    done_galerie = False
    
    #vars carres
    pushed_at_least_one = False
    nombre_carres_done = 0
    checked_last_carre = False
    can_bypass_lecture = False
    carres = {}
    
    done_carres = False

    released_everything = False

    def __init__(self):
        # default buffer size like teensy (TODO: voir si à garder pour stm32?)
        super().__init__("enac_strat")

        #state machine

        ### début partie générée automatiquement

        #generated using graph_compil v0.1
        RecalageC = State("RecalageC")
        RecalageB = State("RecalageB")
        RecalageA = State("RecalageA")
        HasRentreAuBercail = State("HasRentreAuBercail")
        Fin = State("Fin")
        PushCarre = State("PushCarre")
        GoCarre = State("GoCarre")
        HasDroppedStatuette = State("HasDroppedStatuette")
        AtVitrine = State("AtVitrine")
        HasDroppedReplique = State("HasDroppedReplique")
        HasTurnedAroundReplique = State("HasTurnedAroundReplique")
        HasRecupStatuette = State("HasRecupStatuette")
        AtStatuette = State("AtStatuette")
        IsRentringAuBercail = State("IsRentringAuBercail")
        Init = State("Init", self.on_init)
        tr453775270ToIsRentringAuBercail = Transition("tr453775270ToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.quinze_dernieres_secondes)
        AtStatuette.add_transition(tr453775270ToIsRentringAuBercail)
        HasRecupStatuette.add_transition(tr453775270ToIsRentringAuBercail)
        HasTurnedAroundReplique.add_transition(tr453775270ToIsRentringAuBercail)
        HasDroppedReplique.add_transition(tr453775270ToIsRentringAuBercail)
        AtVitrine.add_transition(tr453775270ToIsRentringAuBercail)
        HasDroppedStatuette.add_transition(tr453775270ToIsRentringAuBercail)
        GoCarre.add_transition(tr453775270ToIsRentringAuBercail)
        PushCarre.add_transition(tr453775270ToIsRentringAuBercail)
        tr25103878ToFin = Transition("tr25103878ToFin", Fin, self.tout_flinguer, self.is_fin)
        AtStatuette.add_transition(tr25103878ToFin)
        HasRecupStatuette.add_transition(tr25103878ToFin)
        HasTurnedAroundReplique.add_transition(tr25103878ToFin)
        HasDroppedReplique.add_transition(tr25103878ToFin)
        AtVitrine.add_transition(tr25103878ToFin)
        HasDroppedStatuette.add_transition(tr25103878ToFin)
        GoCarre.add_transition(tr25103878ToFin)
        PushCarre.add_transition(tr25103878ToFin)
        IsRentringAuBercail.add_transition(tr25103878ToFin)
        HasRentreAuBercail.add_transition(tr25103878ToFin)
        IsRentringAuBercailToHasRentreAuBercail = Transition("IsRentringAuBercailToHasRentreAuBercail", HasRentreAuBercail, self.things_todo_at_bercail, self.is_at_bercail)
        IsRentringAuBercail.add_transition(IsRentringAuBercailToHasRentreAuBercail)
        InitToAtStatuette = Transition("InitToAtStatuette", AtStatuette, self.go_recup_statuette, self.is_tirette_activee)
        Init.add_transition(InitToAtStatuette)
        AtStatuetteToHasRecupStatuette = Transition("AtStatuetteToHasRecupStatuette", HasRecupStatuette, self.recup_statuette, self.is_at_statuette)
        AtStatuette.add_transition(AtStatuetteToHasRecupStatuette)
        tr581141130ToHasTurnedAroundReplique = Transition("tr581141130ToHasTurnedAroundReplique", HasTurnedAroundReplique, self.turn_around_replique, self.has_gotten_statuette)
        HasRecupStatuette.add_transition(tr581141130ToHasTurnedAroundReplique)
        tr86346170ToHasDroppedReplique = Transition("tr86346170ToHasDroppedReplique", HasDroppedReplique, self.drop_replique, self.has_turned_around_replique)
        HasTurnedAroundReplique.add_transition(tr86346170ToHasDroppedReplique)
        HasDroppedRepliqueToAtVitrine = Transition("HasDroppedRepliqueToAtVitrine", AtVitrine, self.go_vitrine, self.has_dropped_replique)
        HasDroppedReplique.add_transition(HasDroppedRepliqueToAtVitrine)
        AtVitrineToHasDroppedStatuette = Transition("AtVitrineToHasDroppedStatuette", HasDroppedStatuette, self.drop_statuette, self.is_at_vitrine)
        AtVitrine.add_transition(AtVitrineToHasDroppedStatuette)
        HasDroppedStatuetteToRecalageA = Transition("HasDroppedStatuetteToRecalageA", RecalageA, self.recalage_a, self.has_dropped_stat)
        HasDroppedStatuette.add_transition(HasDroppedStatuetteToRecalageA)
        RecalageAToRecalageB = Transition("RecalageAToRecalageB", RecalageB, self.recalage_b, self.has_recale_a)
        RecalageA.add_transition(RecalageAToRecalageB)
        RecalageBToRecalageC = Transition("RecalageBToRecalageC", RecalageC, self.recalage_c, self.has_recale_b)
        RecalageB.add_transition(RecalageBToRecalageC)
        RecalageCToGoCarre = Transition("RecalageCToGoCarre", GoCarre, self.go_carre, self.has_recale_c)
        RecalageC.add_transition(RecalageCToGoCarre)
        GoCarreToPushCarre = Transition("GoCarreToPushCarre", PushCarre, self.push_carre, self.is_at_carre)
        GoCarre.add_transition(GoCarreToPushCarre)
        PushCarreToIsRentringAuBercail = Transition("PushCarreToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_waited_some_time)
        PushCarre.add_transition(PushCarreToIsRentringAuBercail)
        self.EnacStrat = StateMachine(Init)
        ### fin partie générée automatiquement

        #paramétrage ROS
        self.ros_periph_pub = self.create_publisher(PeriphValue, '/peripherals', 10)
        self.ros_diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.ros_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_pub = self.create_publisher(SetNavigation, '/navigation', 10)
        self.publisher_map = self.create_publisher(
            TransformStamped, 'carte_coins', 10)
        
        self.ros_odom_sub = self.create_subscription(Odometry, '/odom', self.on_ros_odom, 10)
        self.ros_periph_sub = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph, 10)

        self.EnacStrat.start()

        print(self)
        self.send_periph_msg("d", self.score)
        self.send_all_diags()

    def send_tf_map_corners(self):
        msg_out_r = TransformStamped()
        msg_out_r.header.frame_id = "map"
        msg_out_r.child_frame_id = "odom"
        msg_out_r.transform.translation.x = self.x
        msg_out_r.transform.translation.y = self.y
        msg_out_r.transform.translation.z = 0.0
        [qxr, qyr, qzr, qwr] = quaternion_from_euler(
            0, 0, self.theta)

        msg_out_r.transform.rotation.x = qxr
        msg_out_r.transform.rotation.y = qyr
        msg_out_r.transform.rotation.z = qzr
        msg_out_r.transform.rotation.w = qwr
        self.publisher_map.publish(msg_out_r)

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
        return "Strategy: state: "+str(self.EnacStrat.state)+" x: "+str(self.x)+" y: "+str(self.y)+" theta: "+str(self.theta)+(" match unstarted "if self.chrono == 0 else " chrono: "+str(time.time() - self.chrono))
    
    def color_is_jaune(self):
        return (self.periphs.get("co", 0) == 0) #retourne jaune par défaut

    def send_all_diags(self):
        if self.chrono == 0:
            self.send_diagnostic(DiagnosticStatus.OK, "ST: match", "Match has not started")
        elif self.chrono != 0 and (time.time() - self.chrono) > self.end:
            self.send_diagnostic(DiagnosticStatus.ERROR, "ST: match", "Match has ended, strategy is blocked")
        else:
            self.send_diagnostic(DiagnosticStatus.OK if ((time.time() - self.chrono) < self.end - 15) else DiagnosticStatus.WARN, "ST: match", f"{str(time.time() - self.chrono)[:7]} seconds have passed")
        self.send_diagnostic(DiagnosticStatus.STALE, "ST: state", f"{str(self.EnacStrat.state)}")
        self.send_diagnostic(DiagnosticStatus.STALE, "ST: score", f"{str(self.score)} - {str(self.what_done)}")
        self.send_diagnostic(DiagnosticStatus.STALE, "ST: odom", "x:{:.3f} y:{:.3f} theta:{:1f}".format(self.x, self.y, math.degrees(self.theta)))
        self.send_diagnostic(DiagnosticStatus.STALE, "ST: color", f"{('yellow' if self.color_is_jaune() else 'violet')}")
        self.send_tf_map_corners()

    def on_ros_periph(self, msg):
        if (msg.header.frame_id == "stm32"):#to prevent looping from self messages
            id = msg.periph_name[:2]
            cmd = int(msg.value)
            print(f"ros_periph [{id}] {cmd}")
            self.periphs[id] = cmd
            self.check_transitions()

    def on_ros_odom(self, msg):
        if (msg.header.frame_id == "map" and msg.child_frame_id == "odom"):
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.theta = z_euler_from_quaternions(qx, qy, qz, qw)
            self.vlin = msg.twist.twist.linear.x
            self.vtheta = msg.twist.twist.angular.z
            print(f"ros_odom x:{self.x} y:{self.y} theta:{math.degrees(self.theta)}")
            self.check_transitions()

    def is_at_goal(self, vlin_tol, vtheta_tol, distmax_m, theta_tol):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - self.goalx)**2 + (self.y - self.goaly)**2) <= distmax_m and
                abs(self.theta%(2*math.pi) - self.goaltheta%(2*math.pi)) < theta_tol) #TODO: add condition sur theta si utile

    def check_goal(self):
        return self.is_at_goal(0.001, 0.001, 0.1, math.radians(5))

    def is_at_pos(self, vlin_tol, vtheta_tol, distmax_m, goalx, goaly, goaltheta, theta_tol):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - goalx)**2 + (self.y - goaly)**2) <= distmax_m and
                abs(self.theta%(2*math.pi) - goaltheta%(2*math.pi)) < theta_tol) #TODO: add condition sur theta si utile

    def update_score(self, act_name, pts):
        self.score += pts
        self.what_done += (act_name + " ")
        self.update_score_display()

    def check_transitions(self):
        if not self.dont_do_shit_anymore:
            try:
                old_state = self.EnacStrat.state
                self.EnacStrat.check_transitions()
                if self.EnacStrat.state != old_state:
                    print(self)
            except Exception as e:
                print("Strategy: crap in transition")
                print(e)
        else:
            print("Strategy blocked: end of match")
        self.send_all_diags()
    
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
    
    def update_score_display(self):
        self.send_periph_msg("d", self.score)

    def on_init(self):
        pass

    def tout_flinguer(self):
        self.send_periph_msg("d", self.score)
        self.send_nav_msg(0, 0, 0, 0)
        self.dont_do_shit_anymore = True
        self.send_periph_msg("p1", 0)
        self.send_periph_msg("p2", 0)

    
    def is_fin(self):
        return ((time.time() - self.chrono) > self.end - 0.5)


    def go_recup_statuette(self):
        print("Strategy: tirette activée")
        self.chrono = time.time()
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.39, 0.39, math.radians(45))
            #shlag self.send_nav_msg(1, 0.6, 1.14, math.radians(0))
        else:
            self.send_nav_msg(1, 3.0-0.39, 0.39, math.radians(45+90))
            #shlag self.send_nav_msg(1, 3.0-0.6, 1.14, math.radians(180))

    def is_tirette_activee(self):
        return (self.periphs.get("TI", None) is not None)

    def recup_statuette(self):
        self.send_periph_msg("mj", 0) # récup statuette arrière
        pass
        #TODO: il faut faire qqch d'autre ici?

    def is_at_statuette(self):
        return self.check_goal()

    def turn_around_replique(self):
        #ajouter les points car la statuette a été enlevée
        self.update_score("enlv_stat", 5)
        #se retourner pour déposer la réplique
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.38, 0.38, math.radians(225))
        else:
            self.send_nav_msg(1, 3.0-0.38, 0.38, math.radians(-45))

    def has_gotten_statuette(self):
        return (self.periphs.get("mr", None) == 16) #FIXME: coder l'état de neutre avec statuette en bas niveau et le renseigner ici

    def drop_replique(self):
        self.send_periph_msg("mk", 0) #drop stat/repl avant

    def has_turned_around_replique(self):
        return self.check_goal()

    def go_vitrine(self):
        #ajouter les points car réplique droppée
        self.update_score("drop_repl", 10)
        #aller à la vitrine
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.24, 1.83, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-0.24, 1.83, math.radians(-90))

    def has_dropped_replique(self):
        return (self.periphs.get("mv", None) == 1) #la state_machine avant est revenue en position neutre sans charge

    def drop_statuette(self):
        self.send_periph_msg("ml", 0) #drop stat arriere
        self.update_score("stat_vitr", 15)

    def is_at_vitrine(self):
        return self.check_goal()
    
    def recalage_a(self):
        pass

    def has_dropped_stat(self):
        return (self.periphs.get("mr", None) == 1)

    def recalage_b(self):
        pass

    def has_recale_a(self):
        return True
    
    def recalage_c(self):
        pass

    def has_recale_b(self):
        return True

    def go_carre(self):
        #if self.color_is_jaune():
        #    self.send_nav_msg(1, 0.8525, 0.2, math.radians(180))
        #else:
        #    self.send_nav_msg(1, 3.0-0.8525, 0.2, math.radians(180))
        pass

    def has_recale_c(self):
        return True

    def push_carre(self):
        #self.update_score("atleast1_carre", 5)
        #self.update_score("carre_1", 5)
        #self.send_periph_msg("s1", 60)
        #self.time_push = time.time()
        pass

    def is_at_carre(self):
        #return self.check_goal()
        return True

    def has_waited_some_time(self):
        if ((time.time() - self.time_push) > 1.0):
            self.send_periph_msg("s1", 130)
            return True
        else:
            return False

    # version normale plus bas
    def go_palet_rouge(self):
        self.done_galerie = True
        if self.color_is_jaune():
            self.send_nav_msg(1, 1.0, 0.6675, math.radians(47.34))
        else:
            self.send_nav_msg(1, 3.0-1.0, 0.6675, math.radians(180-47.34))

    def is_prio_galerie(self):
        return (True ) and (self.prio_galerie == True) #remplacer le premier True par has_recale_c

    def recup_rouge_stocker(self):
        self.send_periph_msg("ma", 0)
        self.send_periph_msg("mc", 0)

    def is_at_palet_rouge(self):
        return self.check_goal()

    def put_back_rouge(self):
        self.send_periph_msg("mf", 0)

    def has_stored_rouge(self):
        return (self.periphs.get("sc", None) == 1)

    def go_palet_vert(self):
        if self.color_is_jaune():
            self.send_nav_msg(1, 1.0, 1.205, math.radians(45))
        else:
            self.send_nav_msg(1, 3.0-1.0, 1.205, math.radians(180-45))

    def has_backhand_rouge(self):
        return (self.periphs.get("mr", None) == 3)

    def recup_vert_stocker(self):
        self.send_periph_msg("ma", 0)
        self.send_periph_msg("mc", 0)

    def is_at_palet_vert(self):
        return self.check_goal()

    def go_palet_bleu(self):
        if self.color_is_jaune():
            self.send_nav_msg(1, 1.0, 1.455, math.radians(0))
        else:
            self.send_nav_msg(1, 3.0-1.0, 1.455, math.radians(180))

    def has_stored_vert(self):
        return (self.periphs.get("sc", None) == 1)

    def recup_bleu(self):
        self.send_periph_msg("ma", 0)

    def is_at_palet_bleu(self):
        return self.check_goal()

    def go_galerie_rouge(self):
        if self.color_is_jaune():
            self.send_nav_msg(1, 1.05, 1.8, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-1.05, 1.8, math.radians(-90))

    def has_recup_bleu(self):
        return (self.periphs.get("mv", None) == 3)

    def depot_rouge_arriere(self):
        #TODO: code bas-niveau
        self.send_periph_msg("mh", 0) #placeholder pour dépôt arrière sur galerie ->force les AX12 comme un bourrin
        self.send_periph_msg("mf", 0)

    def is_at_galerie_rouge_retourne(self):
        return self.check_goal()

    def go_galerie_vert(self):
        self.update_score("gal_rouge", 6)
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.81, 1.8, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-0.81, 1.8, math.radians(-90))

    def has_dropped_rouge(self):
        return (self.periphs.get("mr", None) == 1)

    def destore_drop_vert_arriere(self):
        self.send_periph_msg("mh", 0) #placeholder pour dépôt arrière sur galerie ->force les AX12 comme un bourrin

    def is_at_galerie_vert_retourne(self):
        return self.check_goal()

    def store_bleu_from_front_hand_and_go_galerie_bleu(self):
        self.update_score("gal_vert", 6)
        self.send_periph_msg("mc", 0)
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.57, 1.8, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-0.57, 1.8, math.radians(-90))

    def has_dropped_vert(self):
        return (self.periphs.get("mr", None) == 1)

    def destore_drop_bleu_arriere(self):
        self.send_periph_msg("mf", 0)
        self.send_periph_msg("mh", 0) #placeholder pour dépôt arrière sur galerie ->force les AX12 comme un bourrin
        self.update_score("gal_bleu", 6)

    def is_at_galerie_bleu(self):
        return self.check_goal()

    def do_nothing(self):
        pass

    def has_dropped_bleu(self):
        return (self.periphs.get("mr", None) == 1)

    def go_carres(self):
        self.done_carres = True
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.6675, 0.2, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-0.6675, 0.2, math.radians(-90))

    def pas_deja_fait_carres(self):
        return not self.done_carres

    def go_bercail(self):
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.2, 1.16, math.radians(180))
        else:
            self.send_nav_msg(1, 3.0-0.2, 1.16, 0)

    def has_deja_fait_carres(self):
        return self.done_carres

    def is_prio_carres(self):
        return (True) and (self.prio_galerie == False)#TODO: remplacer with has_recale_c

    def se_coller_au_mur_deployer_poelon(self):
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.6675, 0.2, math.radians(0))
        else:
            self.send_nav_msg(1, 3.0-0.6675, 0.2, math.radians(0))
        self.send_periph_msg("s1", 100)#TODO: remplacer valeur au pif par vraie valeur

    def is_devant_carres(self):
        return self.is_at_goal(0.001, 0.001, 0.2) #seuil relevé car nav pourrie atm

    def lire_carre_si_besoin(self):
        self.checked_last_carre = False
        self.periphs.pop("LR", None)
        if (self.nombre_carres_done == 0 or self.nombre_carres_done == 3):
            self.send_periph_msg("r", 0)
            self.can_bypass_lecture = False
        else:
            self.can_bypass_lecture = True
        

    def is_au_mur(self):
        return True

    def pousser_si_besoin(self):
        if not self.can_bypass_lecture:
            should_pousse = self.periphs.get("LR", None) == 1
            self.carres[self.nombre_carres_done] = should_pousse
            if should_pousse:
                self.send_periph_msg("s1", 60) #placeholder, peut être avoir repli auto?
                if not self.pushed_at_least_one:
                    self.pushed_at_least_one = True
                    self.update_score("atleast1_carre", 5)
                self.update_score(f"carre_{self.nombre_carres_done}", 5)
        else:
            should_pousse = {
                1: True,
                2: not self.carres.get(0, 0),
                4: not self.carres.get(3, 0),
                5: not self.carres.get(3, 0),
                6: self.carres.get(3, 0)
            }.get(self.nombre_carres_done, False)
            if should_pousse:
                self.send_periph_msg("s1", 90)
                if not self.pushed_at_least_one:
                    self.pushed_at_least_one = True
                    self.update_score("atleast1_carre", 5)
                self.update_score(f"carre_{self.nombre_carres_done}", 5)

        self.nombre_carres_done += 1
        
        self.checked_last_carre = True

    def has_lu_carre_si_besoin(self):
        return (self.periphs.get("LR", None) is not None) or self.can_bypass_lecture

    def go_prochain_carre(self):
        self.send_periph_msg("s1", 110) #replier poelon en position lecture au cas où
        if self.color_is_jaune():
            pass
        else:
            pass
        #self.send_nav_msg(3, ) #nav3: suivi mur, si ça marche pas repasser nav 1

    def has_checked_carre(self):
        return self.checked_last_carre

    def is_at_prochain(self):
        return True#placeholder

    def quitter_mur_rentrer_poelon(self):
        self.send_periph_msg("s1", 130) #replier poelon en position repliée pour se barrer

    def tous_carres_lus(self):
        if self.carres.get(3, None) == True:
            return (self.nombre_carres_done >= 7)
        else:
            return (self.nombre_carres_done >= 6)

    def has_quitte_mur(self):
        return True

    def has_deja_fait_galerie(self):
        return self.done_galerie


    def go_palet_rouge_depuis_mur(self):
        pass

    def pas_deja_fait_galerie(self):
        return not self.done_galerie

    def quinze_dernieres_secondes(self):
        return ((time.time() - self.chrono) > self.end - 15.0)

    def things_todo_at_bercail(self):
        #être sûr d'arrêter le robot
        self.update_score("bercail", 20)
        self.send_nav_msg(0, self.x, self.y, self.theta)
        self.dont_do_shit_anymore = True
        self.send_cmd_vel(0.0, 0.0)
        #déposer tous les palets dans les mains si il y en a
        if self.periphs.get("mv", None) == 3:
            self.send_periph_msg("mg", 0)
            self.score += 1
        if self.periphs.get("mr", None) == 3:
            self.send_periph_msg("mh", 0)
            self.score += 1
        self.update_score_display()
        self.send_periph_msg("p1", 0)
        self.send_periph_msg("p2", 0)

    def is_at_bercail(self):
        if self.color_is_jaune():
            return (self.is_at_pos(0.001, 0.001, 0.1, 0.2, 1.16, math.radians(180), math.radians(5)))
        else:
            return (self.is_at_pos(0.001, 0.001, 0.1, 3.0- 0.2, 1.16, 0, math.radians(5)))

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