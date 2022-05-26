from lidar_location.Triangulation import Triangulation
from lidar_location.Amalgame_list import Amalgame_list
from lidar_location.Point import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from lidar_location.Triangle import Triangle
from geometry_msgs.msg import TransformStamped, Transform

import math
import timeit

import numpy as np

from interfaces_enac.msg import _periph_value, _pid

PeriphValue = _periph_value.PeriphValue
Pid = _pid.Pid


def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


class lidarlocation(Node):
    def __init__(self, name='Lidar_location'):
        super().__init__(name)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        
        self.ros_periph_listener = self.create_subscription(PeriphValue, '/peripherals', self.on_ros_periph_cmd, 10)
        self.publisher_ = self.create_publisher(LaserScan, 'filtered_scan', 10)
        self.publisher_map = self.create_publisher(
            TransformStamped, 'carte_coins', 10)
        self.publish_position = self.create_publisher(
            TransformStamped, 'robot_position', 10)
        self.publish_position_alert = self.create_publisher(
            Bool, 'alert_robot_position_instable', 10)
        ## CRITICAL CODE AHEAD : DO NOT TOUCH
        self.publisher_proximity_warning = self.create_publisher(Float32, 'front_distance', 10)
        self.proximity_distance = 0.35 #CRITICAL: Trigger distance of the proximity sensor
        self.proximity_threshold = 3  # CRITICAL: number of times after which an intrusion is detected before a proximity warning is raised
        self.proximity_threshold_counter = 0 # CRITICAL: Counts the number of detections before raising a stop
        ## END OF CRITICAL CODE
        self.timer = self.create_timer(1, self.test)
        self.positions = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]] # Defines a list of positions to be evened out
        self.pos_counter = 0 #Incremented at every new position to return the median value
        self.last_good = 0 # Used in determiner_position for abborhent values
        self.start = timeit.default_timer()
        self.sens_table = 1 # 0 = jaune, 1 = violet

    def on_ros_periph_cmd(self, msg):
        if (msg.header.frame_id == "stm32"):#to prevent looping from self messages
            id = msg.periph_name[:2]
            cmd = msg.value
            if id == "co":
                if int(cmd) == 0:
                    self.sens_table = 0
                else:
                    self.sens_table = 1

    def test(self):
        msg_out = TransformStamped()
        msg_out.header.frame_id = "map"
        msg_out.child_frame_id = "map_3000_2000"
        msg_out.transform.translation.x = 3.0
        msg_out.transform.translation.y = 2.0
        msg_out.transform.translation.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(
            0, 0, 0)

        msg_out.transform.rotation.x = qx
        msg_out.transform.rotation.y = qy
        msg_out.transform.rotation.z = qz
        msg_out.transform.rotation.w = qw
        self.publisher_map.publish(msg_out)

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

    def generate_filtered_message(self, message, filtered_data):
        out = message
        for i in range(0, len(filtered_data)):
            out.ranges[i] = filtered_data[i]
        return out

    def filter_out(self, message):
        out = []
        for i in range(0, len(message.ranges)):
            if message.ranges[i] is not None:
                if message.ranges[i] > 3.3:
                    out.append(0)
                else:
                    out.append(message.ranges[i])
            else:
                out.append(message.ranges[i])
        return out

    def send_position(self, position):
        msg_out = TransformStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "map"
        msg_out.child_frame_id = "laser"
        msg_out.transform.translation.x = position[0]
        msg_out.transform.translation.y = position[1]
        msg_out.transform.translation.z = 0.0
        [qx, qy, qz, qw] = quaternion_from_euler(
            0, 0, position[2])

        msg_out.transform.rotation.x = qx
        msg_out.transform.rotation.y = qy
        msg_out.transform.rotation.z = qz
        msg_out.transform.rotation.w = qw
        self.publish_position.publish(msg_out)

    def listener_callback(self, msg):
        self.start = timeit.default_timer()
        
        ## CRITICAL CODE AHEAD : DO NOT TOUCH ##
        # PROXIMITY WARNING SYSTEM
        # Detects any object closer than self.proximity_distance and triggers a STOP message
        detection_angle = 56 # in number of points 0.8 degrees between 2 points 56 val = 45 degs
        distances_of_elements = []
        for idx, distance in enumerate(msg.ranges):
            if distance > 0.10 and (idx < detection_angle or idx > (len(msg.ranges) - detection_angle)):
                distances_of_elements.append(distance)
                self.proximity_threshold_counter += 1 # This counter eleiminates artefacts
                #if self.proximity_threshold_counter < self.proximity_threshold:
                # SENDS A PROXIMITY STOP
        msg_prox = Float32()
        closest_element = 15
        for el in distances_of_elements:
            if el < closest_element and el > 0.10:
                closest_element = el
        try:
            msg_prox.data = closest_element 
        except AssertionError:
            print(f"erreur : type incorrect pour msg_prox.data {closest_element}")
            msg_prox.data = 5.0
        self.publisher_proximity_warning.publish(msg_prox)
        print("AIRPROX", distance)

        self.proximity_threshold_counter -= 1
        ## END OF CRITICAL CODE ##
        
        
        msg_out = self.generate_filtered_message(msg, self.filter_out(msg))
        # msg_out = self.generate_filtered_message(
        #    msg, self.filter_out(self.generate_fake(msg)))
        # msg_out = msg
        triangulation = Triangulation(msg_out)

        # DEPLACER LE CODE
        is_poteau_on_right = False

        objl = Amalgame_list(msg_out)
        #[(p.relative_center.pos_x, p.relative_center.pos_y) for p in objl.list_obj]
        # self.get_logger().info("Une list dobj:")
        # filter objcets by distance between points

        def calculate_distance_xy(x1, y1, x2, y2):
            return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

        # TODO : improve loop
        centers = [p.relative_center for p in objl.list_obj]
        valid_triangles = []
        valid_centers = []
        nb_pts = len(centers)
        last_turn = False  # switch to True when finding one valid point, and when calculating distance we find the two other points

        for i in range(nb_pts):
            finding_b = False
            finding_c = False
            if last_turn:
                valid_triangles.append(valid_centers)
                break
            valid_centers = []
            for j in range(i+1, nb_pts):
                cur_dist = calculate_distance_xy(
                    centers[i].pos_x, centers[i].pos_y, centers[j].pos_x, centers[j].pos_y)
                if (cur_dist > 1.7 and cur_dist < 2.0):
                    finding_b = True
                    valid_centers.append(centers[j])
                elif (cur_dist > 3.1 and cur_dist < 3.3):
                    finding_c = True
                    valid_centers.append(centers[j])
                if finding_c and finding_c:
                    last_turn = True
                    valid_centers.append(centers[i])
        # removes duplicates
        valid_centers = list(dict.fromkeys(valid_centers))

        i = 0
        j = 1
        k = 2
        tri_list = []
        while i < len(valid_centers) - 2:
            j = i + 1
            while j < len(valid_centers) - 1:
                k = j + 1
                while k < len(valid_centers):
                    if valid_centers[i] != valid_centers[j] and valid_centers[i] != valid_centers[k] and valid_centers[j] != valid_centers[k]:
                        tri_list.append(Triangle(
                            valid_centers[i], valid_centers[j], valid_centers[k]))
                    k += 1
                j += 1
            i += 1

        for tri in tri_list:
            position = self.determiner_position(tri)
            print("++++++++++++++++++++++++++++++")
            if position[0] > 0 and position[1] > 0:
                #print(position)
                self.send_position(position)
                calc = timeit.default_timer() - self.start
                #print(calc)
                print(position[0], ";", position[1], ";", position[2])
        
        # Trier la liste par angle modulo pi
        list_pts = []
        msg_obj = msg_out

        self.publisher_.publish(msg_obj)

    def mean_position(self):
        length_l = len(self.positions)
        mean_x = 0
        mean_y = 0
        for i in range(0, length_l):
            mean_x += self.positions[i][0]
            mean_y += self.positions[i][1]
        
        return [mean_x/length_l, mean_y/length_l]

    def determiner_position(self, tri):
        x = 0
        y = 0
        x2 = 0
        y2 = 0

        # Tri des points par angle
        tri.pt_list = sorted(tri.pt_list, key=Point.get_angle)

        k = 0
        i = 0
        j = 0
        
        # détermination des bons indices grace aux petits cotés
        if(calculate_distance_xy(
                tri.pt_list[0].pos_x, tri.pt_list[0].pos_y, tri.pt_list[1].pos_x, tri.pt_list[1].pos_y) > 1.6 and calculate_distance_xy(
                tri.pt_list[0].pos_x, tri.pt_list[0].pos_y, tri.pt_list[1].pos_x, tri.pt_list[1].pos_y) < 1.9):
            j = 0
            i = 1
            k = 2
        elif(calculate_distance_xy(
                tri.pt_list[1].pos_x, tri.pt_list[1].pos_y, tri.pt_list[2].pos_x, tri.pt_list[2].pos_y) > 1.6 and calculate_distance_xy(
                tri.pt_list[1].pos_x, tri.pt_list[1].pos_y, tri.pt_list[2].pos_x, tri.pt_list[2].pos_y) < 1.9):
            j = 1
            i = 2
            k = 0

        elif(calculate_distance_xy(
                tri.pt_list[2].pos_x, tri.pt_list[2].pos_y, tri.pt_list[0].pos_x, tri.pt_list[0].pos_y) > 1.6 and calculate_distance_xy(
                tri.pt_list[2].pos_x, tri.pt_list[2].pos_y, tri.pt_list[0].pos_x, tri.pt_list[0].pos_y) < 1.9):
            j = 2
            i = 0
            k = 1
        else:
            j = 0
            i = 1
            k = 2   
            
        teta = math.pi/2 - get_beta(tri.pt_list[j], tri.pt_list[i])
        phi = math.pi/2 - \
            get_gamma(tri.pt_list[i], tri.pt_list[j],
                    get_beta(tri.pt_list[i], tri.pt_list[j]))
        x = math.cos(teta)*tri.pt_list[j].distance# - 0.1 #COrrection chelou
        x2 = math.cos(phi)*tri.pt_list[i].distance #- 0.1
        y = math.sin(teta)*tri.pt_list[j].distance# + 0.05
        y2 = 1.95 - math.sin(phi)*tri.pt_list[i].distance

        # voir ou qu'il pense qu'elles sont les balises
        msg_bal_1 = TransformStamped()
        msg_bal_1.header.frame_id = "laser"
        msg_bal_1.child_frame_id = "balise1"
        msg_bal_1.transform.translation.x = tri.pt_list[i].distance * math.cos(
            tri.pt_list[i].angle)
        msg_bal_1.transform.translation.y = tri.pt_list[i].distance * math.sin(
            tri.pt_list[i].angle)
        msg_bal_1.transform.translation.z = 0.0
        [qx1, qy1, qz1, qw1] = quaternion_from_euler(
            0, 0, 0)

        msg_bal_1.transform.rotation.x = qx1
        msg_bal_1.transform.rotation.y = qy1
        msg_bal_1.transform.rotation.z = qz1
        msg_bal_1.transform.rotation.w = qw1
        self.publisher_map.publish(msg_bal_1)

        msg_bal_2 = TransformStamped()
        msg_bal_2.header.frame_id = "laser"
        msg_bal_2.child_frame_id = "balise2"
        msg_bal_2.transform.translation.x = tri.pt_list[j].distance * math.cos(
            tri.pt_list[j].angle)
        msg_bal_2.transform.translation.y = tri.pt_list[j].distance * math.sin(
            tri.pt_list[j].angle)
        msg_bal_2.transform.translation.z = 0.0
        [qx2, qy2, qz2, qw2] = quaternion_from_euler(
            0, 0, 0)

        msg_bal_2.transform.rotation.x = qx2
        msg_bal_2.transform.rotation.y = qy2
        msg_bal_2.transform.rotation.z = qz2
        msg_bal_2.transform.rotation.w = qw2
        self.publisher_map.publish(msg_bal_2)

        msg_bal_3 = TransformStamped()
        msg_bal_3.header.frame_id = "laser"
        msg_bal_3.child_frame_id = "balise3"
        msg_bal_3.transform.translation.x = tri.pt_list[k].distance * math.cos(
            tri.pt_list[k].angle)
        msg_bal_3.transform.translation.y = tri.pt_list[k].distance * math.sin(
            tri.pt_list[k].angle)
        msg_bal_3.transform.translation.z = 0.0
        [qx3, qy3, qz3, qw3] = quaternion_from_euler(
            0, 0, 0)

        msg_bal_3.transform.rotation.x = qx3
        msg_bal_3.transform.rotation.y = qy3
        msg_bal_3.transform.rotation.z = qz3
        msg_bal_3.transform.rotation.w = qw3
        self.publisher_map.publish(msg_bal_3)

        resx = (x + x2)/2
        resy = (y + y2)/2

        #orientation angle
        orientation=0
        if( (y<1) and (1-y)/tri.pt_list[k].distance<1 and (1-y)/tri.pt_list[k].distance>-1):
            alpha= math.asin((1-y)/tri.pt_list[k].distance)
            orientation=2*math.pi-(tri.pt_list[k].angle-alpha)   
                
        elif((y>1) and (1-y)/tri.pt_list[k].distance<1 and (1-y)/tri.pt_list[k].distance>-1):
            alpha= math.asin((y-1)/tri.pt_list[k].distance)
            orientation=2*math.pi-(tri.pt_list[k].angle+alpha)
        
        # This part managed the algorithms necessary to even out positions and improve stability
        # This is stanted in Diez/Eve ISESA21 Technical projet Part : Amélioration de la précision
        size_of_memory = 5
        
        # This "constant" defines the maximum distance at which the robot will be at the end of a run. 
        # Rn this number is 14cm plus a margin of 6cm        
        max_distance_between_runs = 0.20
        
        # Incertititude message
        incert = Bool()
        incert.data = False

        self.pos_counter += 1
        self.pos_counter = self.pos_counter % size_of_memory # The modulo is the total number of positions to hold in memory

        if self.pos_counter >= len(self.positions):
            self.positions.append([x,y, orientation])
        else:
            self.positions[self.pos_counter] = [x, y, orientation] # adds the latest position to the list ofp ositions
            
        # This part implements the position drop for abborhent values
        self.last_good += 1
        x_in_time = abs(self.positions[(self.pos_counter-1) % size_of_memory][0] - self.positions[self.pos_counter][0]) #distance between x's in time
        y_in_time = abs(self.positions[(self.pos_counter-1) % size_of_memory][1] - self.positions[self.pos_counter][1]) #distance between y's in time
        
        dist_in_time = math.sqrt(x_in_time**2 + y_in_time**2)
        #If its the first value calculated OF we're inside the maximum distance between points
        if self.positions[self.pos_counter-1][0] == 0 or dist_in_time <= self.last_good * max_distance_between_runs:
            self.last_good = 0 #Reset the counter
        else: #Otherwise return the last good position
            x = self.positions[(self.pos_counter-self.last_good) % size_of_memory][0]
            y = self.positions[(self.pos_counter-self.last_good) % size_of_memory][1]
            orientation = self.positions[(self.pos_counter-self.last_good) % size_of_memory][2]
            incert.data = True

        if self.sens_table == 0: #jaune
            x = 3 - x
            y = 3 - y
            
        self.publish_position_alert.publish(incert) # Publishes incertain position
            
        return [x, y, orientation]
        


def get_distance_pt(pt1, pt2):
    return math.sqrt(pt1.distance**2 + pt2.distance**2 - 2*pt1.distance*pt2.distance*math.cos(abs(pt1.angle-pt2.angle)))


def get_beta(pt1, pt2):
    return math.acos(abs((get_distance_pt(pt1, pt2)**2 + pt2.distance**2 - pt1.distance**2) / (2 * get_distance_pt(pt1, pt2) * pt2.distance)))


def get_gamma(pt1, pt2, beta):
    return math.pi - beta - abs(pt2.angle - pt1.angle)


def calculate_distance_xy(x1, y1, x2, y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


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
