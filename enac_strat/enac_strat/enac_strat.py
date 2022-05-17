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
        HasRentreAuBercail = State("HasRentreAuBercail")
        FinCarres = State("FinCarres")
        QuitterMur = State("QuitterMur")
        ProchainCarre = State("ProchainCarre")
        PousserSiBesoin = State("PousserSiBesoin")
        LireCarre = State("LireCarre")
        ColleAuMur = State("ColleAuMur")
        IsRentringAuBercail = State("IsRentringAuBercail")
        DevantCarres = State("DevantCarres")
        FinGalerie = State("FinGalerie")
        HasDroppedBleuGalerie = State("HasDroppedBleuGalerie")
        DevantGalerieBleuRetourne = State("DevantGalerieBleuRetourne")
        HasDroppedVertGalerie = State("HasDroppedVertGalerie")
        DevantGalerieVertRetourne = State("DevantGalerieVertRetourne")
        HasDroppedRougeGalerie = State("HasDroppedRougeGalerie")
        DevantGalerieRougeRetourne = State("DevantGalerieRougeRetourne")
        HasRecupPaletBleu = State("HasRecupPaletBleu")
        DevantPaletBleu = State("DevantPaletBleu")
        HasRecupPaletVertAndStored = State("HasRecupPaletVertAndStored")
        DevantPaletVert = State("DevantPaletVert")
        RougeInBackHand = State("RougeInBackHand")
        HasRecupPaletRougeAndStored = State("HasRecupPaletRougeAndStored")
        DevantPaletRouge = State("DevantPaletRouge")
        HasDroppedStatuette = State("HasDroppedStatuette")
        AtVitrine = State("AtVitrine")
        HasDroppedReplique = State("HasDroppedReplique")
        HasTurnedAroundReplique = State("HasTurnedAroundReplique")
        HasRecupStatuette = State("HasRecupStatuette")
        AtStatuette = State("AtStatuette")
        Init = State("Init", self.on_init)
        InitToAtStatuette = Transition("InitToAtStatuette", AtStatuette, self.go_recup_statuette, self.is_tirette_activee)
        tr139301719ToIsRentringAuBercail = Transition("tr139301719ToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.quinze_dernieres_secondes)
        AtStatuette.add_transition(tr139301719ToIsRentringAuBercail)
        HasRecupStatuette.add_transition(tr139301719ToIsRentringAuBercail)
        HasTurnedAroundReplique.add_transition(tr139301719ToIsRentringAuBercail)
        HasDroppedReplique.add_transition(tr139301719ToIsRentringAuBercail)
        AtVitrine.add_transition(tr139301719ToIsRentringAuBercail)
        HasDroppedStatuette.add_transition(tr139301719ToIsRentringAuBercail)
        DevantPaletRouge.add_transition(tr139301719ToIsRentringAuBercail)
        HasRecupPaletRougeAndStored.add_transition(tr139301719ToIsRentringAuBercail)
        RougeInBackHand.add_transition(tr139301719ToIsRentringAuBercail)
        DevantPaletVert.add_transition(tr139301719ToIsRentringAuBercail)
        HasRecupPaletVertAndStored.add_transition(tr139301719ToIsRentringAuBercail)
        DevantPaletBleu.add_transition(tr139301719ToIsRentringAuBercail)
        HasRecupPaletBleu.add_transition(tr139301719ToIsRentringAuBercail)
        DevantGalerieRougeRetourne.add_transition(tr139301719ToIsRentringAuBercail)
        HasDroppedRougeGalerie.add_transition(tr139301719ToIsRentringAuBercail)
        DevantGalerieVertRetourne.add_transition(tr139301719ToIsRentringAuBercail)
        HasDroppedVertGalerie.add_transition(tr139301719ToIsRentringAuBercail)
        DevantGalerieBleuRetourne.add_transition(tr139301719ToIsRentringAuBercail)
        HasDroppedBleuGalerie.add_transition(tr139301719ToIsRentringAuBercail)
        FinGalerie.add_transition(tr139301719ToIsRentringAuBercail)
        DevantCarres.add_transition(tr139301719ToIsRentringAuBercail)
        FinCarres.add_transition(tr139301719ToIsRentringAuBercail)
        PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, self.quitter_mur_rentrer_poelon, self.quinze_dernieres_secondes)
        PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
        IsRentringAuBercailToHasRentreAuBercail = Transition("IsRentringAuBercailToHasRentreAuBercail", HasRentreAuBercail, self.things_todo_at_bercail, self.is_at_bercail)
        IsRentringAuBercail.add_transition(IsRentringAuBercailToHasRentreAuBercail)
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
        HasDroppedStatuetteToDevantPaletRouge = Transition("HasDroppedStatuetteToDevantPaletRouge", DevantPaletRouge, self.go_palet_rouge, self.is_prio_galerie)
        HasDroppedStatuette.add_transition(HasDroppedStatuetteToDevantPaletRouge)
        tr443085219ToHasRecupPaletRougeAndStored = Transition("tr443085219ToHasRecupPaletRougeAndStored", HasRecupPaletRougeAndStored, self.recup_rouge_stocker, self.is_at_palet_rouge)
        DevantPaletRouge.add_transition(tr443085219ToHasRecupPaletRougeAndStored)
        tr224422110ToRougeInBackHand = Transition("tr224422110ToRougeInBackHand", RougeInBackHand, self.put_back_rouge, self.has_stored_rouge)
        HasRecupPaletRougeAndStored.add_transition(tr224422110ToRougeInBackHand)
        RougeInBackHandToDevantPaletVert = Transition("RougeInBackHandToDevantPaletVert", DevantPaletVert, self.go_palet_vert, self.has_backhand_rouge)
        RougeInBackHand.add_transition(RougeInBackHandToDevantPaletVert)
        tr487924451ToHasRecupPaletVertAndStored = Transition("tr487924451ToHasRecupPaletVertAndStored", HasRecupPaletVertAndStored, self.recup_vert_stocker, self.is_at_palet_vert)
        DevantPaletVert.add_transition(tr487924451ToHasRecupPaletVertAndStored)
        tr226431985ToDevantPaletBleu = Transition("tr226431985ToDevantPaletBleu", DevantPaletBleu, self.go_palet_bleu, self.has_stored_vert)
        HasRecupPaletVertAndStored.add_transition(tr226431985ToDevantPaletBleu)
        DevantPaletBleuToHasRecupPaletBleu = Transition("DevantPaletBleuToHasRecupPaletBleu", HasRecupPaletBleu, self.recup_bleu, self.is_at_palet_bleu)
        DevantPaletBleu.add_transition(DevantPaletBleuToHasRecupPaletBleu)
        tr298394497ToDevantGalerieRougeRetourne = Transition("tr298394497ToDevantGalerieRougeRetourne", DevantGalerieRougeRetourne, self.go_galerie_rouge, self.has_recup_bleu)
        HasRecupPaletBleu.add_transition(tr298394497ToDevantGalerieRougeRetourne)
        tr156138794ToHasDroppedRougeGalerie = Transition("tr156138794ToHasDroppedRougeGalerie", HasDroppedRougeGalerie, self.depot_rouge_arriere, self.is_at_galerie_rouge_retourne)
        DevantGalerieRougeRetourne.add_transition(tr156138794ToHasDroppedRougeGalerie)
        tr1003384989ToDevantGalerieVertRetourne = Transition("tr1003384989ToDevantGalerieVertRetourne", DevantGalerieVertRetourne, self.go_galerie_vert, self.has_dropped_rouge)
        HasDroppedRougeGalerie.add_transition(tr1003384989ToDevantGalerieVertRetourne)
        tr168523427ToHasDroppedVertGalerie = Transition("tr168523427ToHasDroppedVertGalerie", HasDroppedVertGalerie, self.destore_drop_vert_arriere, self.is_at_galerie_vert_retourne)
        DevantGalerieVertRetourne.add_transition(tr168523427ToHasDroppedVertGalerie)
        tr210589313ToDevantGalerieBleuRetourne = Transition("tr210589313ToDevantGalerieBleuRetourne", DevantGalerieBleuRetourne, self.store_bleu_from_front_hand_and_go_galerie_bleu, self.has_dropped_vert)
        HasDroppedVertGalerie.add_transition(tr210589313ToDevantGalerieBleuRetourne)
        tr432180998ToHasDroppedBleuGalerie = Transition("tr432180998ToHasDroppedBleuGalerie", HasDroppedBleuGalerie, self.destore_drop_bleu_arriere, self.is_at_galerie_bleu)
        DevantGalerieBleuRetourne.add_transition(tr432180998ToHasDroppedBleuGalerie)
        HasDroppedBleuGalerieToFinGalerie = Transition("HasDroppedBleuGalerieToFinGalerie", FinGalerie, self.do_nothing, self.has_dropped_bleu)
        HasDroppedBleuGalerie.add_transition(HasDroppedBleuGalerieToFinGalerie)
        FinGalerieToDevantCarres = Transition("FinGalerieToDevantCarres", DevantCarres, self.go_carres, self.pas_deja_fait_carres)
        FinGalerie.add_transition(FinGalerieToDevantCarres)
        FinGalerieToIsRentringAuBercail = Transition("FinGalerieToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_deja_fait_carres)
        FinGalerie.add_transition(FinGalerieToIsRentringAuBercail)
        HasDroppedStatuetteToDevantCarres = Transition("HasDroppedStatuetteToDevantCarres", DevantCarres, self.go_carres, self.is_prio_carres)
        HasDroppedStatuette.add_transition(HasDroppedStatuetteToDevantCarres)
        DevantCarresToColleAuMur = Transition("DevantCarresToColleAuMur", ColleAuMur, self.se_coller_au_mur_deployer_poelon, self.is_devant_carres)
        DevantCarres.add_transition(DevantCarresToColleAuMur)
        ColleAuMurToLireCarre = Transition("ColleAuMurToLireCarre", LireCarre, self.lire_carre_si_besoin, self.is_au_mur)
        ColleAuMur.add_transition(ColleAuMurToLireCarre)
        LireCarreToPousserSiBesoin = Transition("LireCarreToPousserSiBesoin", PousserSiBesoin, self.pousser_si_besoin, self.has_lu_carre_si_besoin)
        LireCarre.add_transition(LireCarreToPousserSiBesoin)
        PousserSiBesoinToProchainCarre = Transition("PousserSiBesoinToProchainCarre", ProchainCarre, self.go_prochain_carre, self.has_checked_carre)
        PousserSiBesoin.add_transition(PousserSiBesoinToProchainCarre)
        ProchainCarreToLireCarre = Transition("ProchainCarreToLireCarre", LireCarre, self.lire_carre_si_besoin, self.is_at_prochain)
        ProchainCarre.add_transition(ProchainCarreToLireCarre)
        PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, self.quitter_mur_rentrer_poelon, self.tous_carres_lus)
        PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
        QuitterMurToFinCarres = Transition("QuitterMurToFinCarres", FinCarres, self.do_nothing, self.has_quitte_mur)
        QuitterMur.add_transition(QuitterMurToFinCarres)
        FinCarresToIsRentringAuBercail = Transition("FinCarresToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_deja_fait_galerie)
        FinCarres.add_transition(FinCarresToIsRentringAuBercail)
        FinCarresToDevantPaletRouge = Transition("FinCarresToDevantPaletRouge", DevantPaletRouge, self.go_palet_rouge_depuis_mur, self.pas_deja_fait_galerie)
        FinCarres.add_transition(FinCarresToDevantPaletRouge)
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

    def is_at_goal(self, vlin_tol, vtheta_tol, distmax_m):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - self.goalx)**2 + (self.y - self.goaly)**2) <= distmax_m) #TODO: add condition sur theta si utile

    def check_goal(self):
        return self.is_at_goal(0.001, 0.001, 0.1)

    def is_at_pos(self, vlin_tol, vtheta_tol, distmax_m, goalx, goaly, goaltheta):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - goalx)**2 + (self.y - goaly)**2) <= distmax_m) #TODO: add condition sur theta si utile

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

    def go_recup_statuette(self):
        print("Strategy: tirette activée")
        self.chrono = time.time()
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.45, 0.45, math.radians(45))
        else:
            self.send_nav_msg(1, 3.0-0.45, 0.45, math.radians(45+90))

    def is_tirette_activee(self):
        return (self.periphs.get("TI", None) is not None)

    def recup_statuette(self):
        #TODO: code bas-niveau
        pass

    def is_at_statuette(self):
        return self.check_goal()

    def turn_around_replique(self):
        #ajouter les points car la statuette a été enlevée
        self.update_score("enlv_stat", 5)
        #se retourner pour déposer la réplique
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.45, 0.45, math.radians(225))
        else:
            self.send_nav_msg(1, 3.0-0.45, 0.45, math.radians(-45))

    def has_gotten_statuette(self):
        return (self.periphs.get("mr", None) == -1) #FIXME: coder l'état de neutre avec statuette en bas niveau et le renseigner ici

    def drop_replique(self):
        #TODO: code bas-niveau
        pass

    def has_turned_around_replique(self):
        return self.check_goal()

    def go_vitrine(self):
        #ajouter les points car réplique droppée
        self.update_score("drop_repl", 10)
        #aller à la vitrine
        if self.color_is_jaune():
            self.send_nav_msg(1, 0.24, 1.8, math.radians(-90))
        else:
            self.send_nav_msg(1, 3.0-0.24, 1.8, math.radians(-90))

    def has_dropped_replique(self):
        return (self.periphs.get("mv", None) == 1) #la state_machine avant est revenue en position neutre sans charge

    def drop_statuette(self):
        #TODO: code bas-niveau
        self.update_score("stat_vitr", 15)

    def is_at_vitrine(self):
        return self.check_goal()

    def go_palet_rouge(self):
        self.done_galerie = True
        if self.color_is_jaune():
            self.send_nav_msg(1, 1.0, 0.6675, math.radians(47.34))
        else:
            self.send_nav_msg(1, 3.0-1.0, 0.6675, math.radians(180-47.34))

    def is_prio_galerie(self):
        return (self.periphs.get("mr", None) == 1) and (self.prio_galerie == True)

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
        return (self.periphs.get("mr", None) == 1) and (self.prio_galerie == False)

    def se_coller_au_mur_deployer_poelon(self):
        pass

    def is_devant_carres(self):
        return True

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
            should_pousse = self.periphs.get("LR", None)
            self.carres[self.nombre_carres_done] = should_pousse
        else:
            pass
        self.nombre_carres_done += 1
        if should_pousse == 1:
            self.send_periph_msg("s1", 90) #placeholder, peut être avoir repli auto?
        self.checked_last_carre = True

    def has_lu_carre_si_besoin(self):
        return (self.periphs.get("LR", None) is not None) or self.can_bypass_lecture

    def go_prochain_carre(self):
        self.send_periph_msg("s1", 110) #replier poelon en position lecture au cas où
        #self.send_nav_msg(3, ) #nav3: suivi mur, si ça marche pas repasser nav 1

    def has_checked_carre(self):
        return self.checked_last_carre

    def is_at_prochain(self):
        return self.check_goal()

    def quitter_mur_rentrer_poelon(self):
        self.send_periph_msg("s1", 130) #replier poelon en position repliée pour se barrer

    def tous_carres_lus(self):
        if self.carres.get(3, None) == 1:
            return (self.nombre_carres_done >= 6)
        else:
            return (self.nombre_carres_done >= 5)

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
            return (self.is_at_pos(0.001, 0.001, 0.1, 0.2, 1.16, math.radians(180)))
        else:
            return (self.is_at_pos(0.001, 0.001, 0.1, 3.0- 0.2, 1.16, 0))

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