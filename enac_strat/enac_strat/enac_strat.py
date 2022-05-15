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

from statemachine import State, Transition, StateMachine

#### fin partie générée automatiquement
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

    score = 4
    chrono = 0
    end = 100

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
        tr443085219ToHasRecupPaletRougeAndStored = Transition("tr443085219ToHasRecupPaletRougeAndStored", HasRecupPaletRougeAndStored, recup_rouge_stocker, is_at_palet_rouge)
        DevantPaletRouge.add_transition(tr443085219ToHasRecupPaletRougeAndStored)
        tr224422110ToRougeInBackHand = Transition("tr224422110ToRougeInBackHand", RougeInBackHand, put_back_rouge, has_stored_rouge)
        HasRecupPaletRougeAndStored.add_transition(tr224422110ToRougeInBackHand)
        RougeInBackHandToDevantPaletVert = Transition("RougeInBackHandToDevantPaletVert", DevantPaletVert, go_palet_vert, has_backhand_rouge)
        RougeInBackHand.add_transition(RougeInBackHandToDevantPaletVert)
        tr487924451ToHasRecupPaletVertAndStored = Transition("tr487924451ToHasRecupPaletVertAndStored", HasRecupPaletVertAndStored, recup_vert_stocker, is_at_palet_vert)
        DevantPaletVert.add_transition(tr487924451ToHasRecupPaletVertAndStored)
        tr226431985ToDevantPaletBleu = Transition("tr226431985ToDevantPaletBleu", DevantPaletBleu, go_palet_bleu, has_stored_vert)
        HasRecupPaletVertAndStored.add_transition(tr226431985ToDevantPaletBleu)
        DevantPaletBleuToHasRecupPaletBleu = Transition("DevantPaletBleuToHasRecupPaletBleu", HasRecupPaletBleu, recup_bleu, is_at_palet_bleu)
        DevantPaletBleu.add_transition(DevantPaletBleuToHasRecupPaletBleu)
        tr298394497ToDevantGalerieRougeRetourne = Transition("tr298394497ToDevantGalerieRougeRetourne", DevantGalerieRougeRetourne, go_galerie_rouge, has_recup_bleu)
        HasRecupPaletBleu.add_transition(tr298394497ToDevantGalerieRougeRetourne)
        tr156138794ToHasDroppedRougeGalerie = Transition("tr156138794ToHasDroppedRougeGalerie", HasDroppedRougeGalerie, depot_rouge_arriere, is_at_galerie_rouge_retourne)
        DevantGalerieRougeRetourne.add_transition(tr156138794ToHasDroppedRougeGalerie)
        tr1003384989ToDevantGalerieVertRetourne = Transition("tr1003384989ToDevantGalerieVertRetourne", DevantGalerieVertRetourne, go_galerie_vert, has_dropped_rouge)
        HasDroppedRougeGalerie.add_transition(tr1003384989ToDevantGalerieVertRetourne)
        tr168523427ToHasDroppedVertGalerie = Transition("tr168523427ToHasDroppedVertGalerie", HasDroppedVertGalerie, destore_drop_vert_arriere, is_at_galerie_vert_retourne)
        DevantGalerieVertRetourne.add_transition(tr168523427ToHasDroppedVertGalerie)
        tr210589313ToDevantGalerieBleuRetourne = Transition("tr210589313ToDevantGalerieBleuRetourne", DevantGalerieBleuRetourne, store_bleu_from_front_hand_and_go_galerie_bleu, has_dropped_vert)
        HasDroppedVertGalerie.add_transition(tr210589313ToDevantGalerieBleuRetourne)
        tr432180998ToHasDroppedBleuGalerie = Transition("tr432180998ToHasDroppedBleuGalerie", HasDroppedBleuGalerie, destore_drop_bleu_arriere, is_at_galerie_bleu)
        DevantGalerieBleuRetourne.add_transition(tr432180998ToHasDroppedBleuGalerie)
        HasDroppedBleuGalerieToFinGalerie = Transition("HasDroppedBleuGalerieToFinGalerie", FinGalerie, do_nothing, has_dropped_bleu)
        HasDroppedBleuGalerie.add_transition(HasDroppedBleuGalerieToFinGalerie)
        FinGalerieToDevantCarres = Transition("FinGalerieToDevantCarres", DevantCarres, go_carres, pas_deja_fait_carres)
        FinGalerie.add_transition(FinGalerieToDevantCarres)
        FinGalerieToIsRentringAuBercail = Transition("FinGalerieToIsRentringAuBercail", IsRentringAuBercail, go_bercail, has_deja_fait_carres)
        FinGalerie.add_transition(FinGalerieToIsRentringAuBercail)
        HasDroppedStatuetteToDevantCarres = Transition("HasDroppedStatuetteToDevantCarres", DevantCarres, go_carres, is_prio_carres)
        HasDroppedStatuette.add_transition(HasDroppedStatuetteToDevantCarres)
        DevantCarresToColleAuMur = Transition("DevantCarresToColleAuMur", ColleAuMur, se_coller_au_mur_deployer_poelon, is_devant_carres)
        DevantCarres.add_transition(DevantCarresToColleAuMur)
        ColleAuMurToLireCarre = Transition("ColleAuMurToLireCarre", LireCarre, lire_carre_si_besoin, is_au_mur)
        ColleAuMur.add_transition(ColleAuMurToLireCarre)
        LireCarreToPousserSiBesoin = Transition("LireCarreToPousserSiBesoin", PousserSiBesoin, pousser_si_besoin, has_lu_carre_si_besoin)
        LireCarre.add_transition(LireCarreToPousserSiBesoin)
        PousserSiBesoinToProchainCarre = Transition("PousserSiBesoinToProchainCarre", ProchainCarre, go_prochain_carre, has_checked_carre)
        PousserSiBesoin.add_transition(PousserSiBesoinToProchainCarre)
        ProchainCarreToLireCarre = Transition("ProchainCarreToLireCarre", LireCarre, lire_carre_si_besoin, is_at_prochain)
        ProchainCarre.add_transition(ProchainCarreToLireCarre)
        PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, quitter_mur_rentrer_poelon, tous_carres_lus)
        PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
        QuitterMurToFinCarres = Transition("QuitterMurToFinCarres", FinCarres, do_nothing, has_quitte_mur)
        QuitterMur.add_transition(QuitterMurToFinCarres)
        FinCarresToIsRentringAuBercail = Transition("FinCarresToIsRentringAuBercail", IsRentringAuBercail, go_bercail, has_deja_fait_galerie)
        FinCarres.add_transition(FinCarresToIsRentringAuBercail)
        FinCarresToDevantPaletRouge = Transition("FinCarresToDevantPaletRouge", DevantPaletRouge, go_palet_rouge_depuis_mur, pas_deja_fait_galerie)
        FinCarres.add_transition(FinCarresToDevantPaletRouge)
        tr992703258ToIsRentringAuBercail = Transition("tr992703258ToIsRentringAuBercail", IsRentringAuBercail, go_bercail, quinze_dernieres_secondes)
        Init.add_transition(tr992703258ToIsRentringAuBercail)
        AtStatuette.add_transition(tr992703258ToIsRentringAuBercail)
        HasRecupStatuette.add_transition(tr992703258ToIsRentringAuBercail)
        HasTurnedAroundReplique.add_transition(tr992703258ToIsRentringAuBercail)
        HasDroppedReplique.add_transition(tr992703258ToIsRentringAuBercail)
        AtVitrine.add_transition(tr992703258ToIsRentringAuBercail)
        HasDroppedStatuette.add_transition(tr992703258ToIsRentringAuBercail)
        DevantPaletRouge.add_transition(tr992703258ToIsRentringAuBercail)
        HasRecupPaletRougeAndStored.add_transition(tr992703258ToIsRentringAuBercail)
        RougeInBackHand.add_transition(tr992703258ToIsRentringAuBercail)
        DevantPaletVert.add_transition(tr992703258ToIsRentringAuBercail)
        HasRecupPaletVertAndStored.add_transition(tr992703258ToIsRentringAuBercail)
        DevantPaletBleu.add_transition(tr992703258ToIsRentringAuBercail)
        HasRecupPaletBleu.add_transition(tr992703258ToIsRentringAuBercail)
        DevantGalerieRougeRetourne.add_transition(tr992703258ToIsRentringAuBercail)
        HasDroppedRougeGalerie.add_transition(tr992703258ToIsRentringAuBercail)
        DevantGalerieVertRetourne.add_transition(tr992703258ToIsRentringAuBercail)
        HasDroppedVertGalerie.add_transition(tr992703258ToIsRentringAuBercail)
        DevantGalerieBleuRetourne.add_transition(tr992703258ToIsRentringAuBercail)
        HasDroppedBleuGalerie.add_transition(tr992703258ToIsRentringAuBercail)
        FinGalerie.add_transition(tr992703258ToIsRentringAuBercail)
        DevantCarres.add_transition(tr992703258ToIsRentringAuBercail)
        FinCarres.add_transition(tr992703258ToIsRentringAuBercail)
        PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, quitter_mur_rentrer_poelon, quinze_dernieres_secondes)
        PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
        IsRentringAuBercailToHasRentreAuBercail = Transition("IsRentringAuBercailToHasRentreAuBercail", HasRentreAuBercail, things_todo_at_bercail, is_at_bercail)
        IsRentringAuBercail.add_transition(IsRentringAuBercailToHasRentreAuBercail)
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
            id = msg.periph_name[:2]
            cmd = int(msg.value)
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
            self.check_transitions()

    def is_at_goal(self, vlin_tol, vtheta_tol, distmax_m):
        return (abs(self.vlin) <= vlin_tol and
                abs(self.vtheta) <= vtheta_tol and
                math.sqrt((self.x - self.goalx)**2 + (self.y - self.goaly)**2) <= distmax_m) #TODO: add condition sur theta si utile

    def check_transitions(self):
        self.send_all_diags()
        try:
            self.EnacStrat.check_transitions()
        except Exception as e:
            print("Strategy: crap in transition")
            print(e)
    
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

    #callbacks des états

    def on_init(self):
        pass

    #callbacks et gardes des transitions
    def go_recup_statuette(self):
        pass
    def is_tirette_active(self):
        pass

    def recup_statuette(self):
        pass
    def is_at_statuette(self):
        pass

    def turn_around_replique(self):
        pass
    def has_gotten_statuette(self):
        pass

    def drop_replique(self):
        pass
    def has_turned_around_replique(self):
        pass

    def go_vitrine(self):
        pass
    def has_dropped_replique(self):
        pass

    def drop_statuette(self):
        pass
    def is_at_vitrine(self):
        pass

    def go_palet_rouge(self):
        pass
    def is_prio_galerie(self):
        pass
    
    #vieux code tout pourri
    def on_start(self):
        print("Strategy: Tirette détectée: start")
        self.chrono = time.time()
        #if (self.periphs.get("co", 0) == 0):
        print("start yellow")
        self.send_nav_msg(1, 0.700, 1.140, 0.0)
        #else:
        #    print("start violet")
        #    self.send_nav_msg(1, 3.0-0.700, 1.140, math.radians(180))

    def on_turn_palet(self):
        print("Strategy: Arrivé destination: Tourner palet")
        #self.send_cmd_vel(-0.001, 0.0029) #je tente des trucs, faire tourner le robot vers la gauche?
        self.send_nav_msg(1, 0.7, 1.140, math.radians(15))
        time.sleep(0.2)
        self.send_periph_msg("ma", 0) #choper un palet au sol
        self.send_periph_msg("mc", 0) # et mettre dans la réserve

    def on_a_chope(self):
        #self.send_cmd_vel(0.0001, 0.0025) #tourner un peu à gauche
        self.send_nav_msg(1, self.x, self.y, math.radians(45))
        print("Strategy: choper second")
        time.sleep(1)
        self.send_periph_msg("mf", 0)
        self.send_periph_msg("ma", 0)
        self.send_periph_msg("mc", 0)

    def on_almost_end(self):
        print("Strategy: returning to home")
        self.score += 20
        self.send_periph_msg("d", self.score)
        #if (self.periphs.get("co", 0) == 0):
        self.send_nav_msg(1, 0.200, 1.200, math.radians(90)) # retourner au bercail
        #else:
        #self.send_nav_msg(1, 3.0-0.2, 1.140, math.radians(90))
    
    def tout_lacher_avant_fin(self):
        self.score += 2
        self.send_periph_msg("mg", 0)
        self.send_periph_msg("mh", 0)

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