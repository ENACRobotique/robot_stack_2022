from lidar_location.Triangulation import Triangulation
from lidar_location.Amalgame_list import Amalgame_list
from lidar_location.Point import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from lidar_location.Triangle import Triangle
from geometry_msgs.msg import TransformStamped, Transform
import math

import numpy as np


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
        self.subscription
        self.publisher_ = self.create_publisher(LaserScan, 'filtered_scan', 10)
        self.publisher_map = self.create_publisher(
            TransformStamped, 'carte_coins', 10)
        self.publish_position = self.create_publisher(
            TransformStamped, 'robot_position', 10)
        self.timer = self.create_timer(1, self.test)

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

    def generate_fake(self, message):
        # TODO : retirer les points de ce vrai msg qui sont pas les 3 poteaux (à la limite en garder 1 ou 2)
        # message.ranges = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3890000581741333, 1.3730000257492065, 1.3580000400543213, 1.3580000400543213, 1.3580000400543213, 1.3890000581741333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9470000267028809, 1.930999994277954, 1.930999994277954, 1.9470000267028809, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.218000054359436, 1.2029999494552612, 1.2489999532699585, 1.2960000038146973, 1.3730000257492065, 1.3890000581741333, 1.3890000581741333, 1.3730000257492065, 3.249000072479248, 3.2339999675750732, 3.2179999351501465, 3.2179999351501465, 0.0, 0.0, 0.0, 0.0, 3.265000104904175, 2.4739999771118164, 2.4119999408721924, 2.4739999771118164, 0.0, 3.2339999675750732, 3.2179999351501465, 3.2179999351501465, 2.490000009536743, 2.4739999771118164, 2.4739999771118164, 2.4590001106262207, 2.4590001106262207, 2.443000078201294, 2.443000078201294, 2.427999973297119, 2.427999973297119, 2.4119999408721924, 2.3970000743865967, 2.3970000743865967, 2.3970000743865967, 2.38100004196167, 2.38100004196167, 2.38100004196167, 2.38100004196167, 2.38100004196167, 2.365999937057495, 2.365999937057495, 2.4119999408721924, 2.4119999408721924, 2.427999973297119, 2.443000078201294, 2.4739999771118164, 2.490000009536743, 2.5209999084472656, 2.5360000133514404, 2.552000045776367, 2.5829999446868896, 2.628999948501587, 2.6600000858306885, 2.690999984741211, 2.7219998836517334, 2.753000020980835, 2.7839999198913574, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.590000033378601, 1.5750000476837158, 1.559000015258789, 1.559000015258789, 1.5750000476837158]

        message.ranges = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.7319999933242798,
            1.715999960899353,
            1.7009999752044678,
            1.7009999752044678,
            1.715999960899353,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.6540000438690186,
            1.6380000114440918,
            1.6230000257492065,
            1.6230000257492065,
            1.6380000114440918,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.715999960899353,
            1.684999942779541,
            1.684999942779541,
            1.684999942779541,
            1.684999942779541,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        return message

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
        # self.get_logger().info(msg.angle_max)
        msg_out = self.generate_filtered_message(msg, self.filter_out(msg))
        # msg_out = self.generate_filtered_message(
        #    msg, self.filter_out(self.generate_fake(msg)))
        # msg_out = msg
        # self.get_logger().info(msg_out.angle_max)
        # triangulation = Amalgame_list(msg_out)
        triangulation = Triangulation(msg_out)
        # self.get_logger().info("-------------")
        # for obj in triangulation.list_obj:
        #    self.get_logger().info(
        #        f"{obj.relative_center}")
        # self.publisher_.publish(msg_out)

        """
        for tri in triangulation.tri_list:
            # self.get_logger().info("------------------------------------------------------")
            # if ((tri.distances[0] > 1.7 and tri.distances[0] < 1.9) and (tri.distances[1] > 3.2 and tri.distances[1] < 3.4) and (tri.distances[2] > 3.2 and tri.distances[2] < 3.4)) or ((tri.distances[0] > 3.2 and tri.distances[0] < 3.4) and (tri.distances[1] > 1.7 and tri.distances[1] < 1.9) and (tri.distances[2] > 3.2 and tri.distances[2] < 3.4)) or ((tri.distances[0] > 3.2 and tri.distances[0] < 3.4) and (tri.distances[1] > 3.2 and tri.distances[1] < 3.4) and (tri.distances[2] > 1.7 and tri.distances[2] < 1.9)) and ((tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.5 and tri.angles[2] < 0.7) or (tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[2] > 0.5 and tri.angles[2] < 0.7 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3) or (tri.angles[2] > 0.5 and tri.angles[2] < 0.7 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3)):
            self.get_logger().info("Un triangle:")
            for pt in tri.pt_list:
                self.get_logger().info("Un point:")
                self.get_logger().info(
                    f"{pt.distance}")
                self.get_logger().info(
                    f"{pt.angle}")
            self.get_logger().info("Les distances:")
            self.get_logger().info(
                f"{tri.distances}")
            self.get_logger().info("Les angles:")
            self.get_logger().info(
                f"{tri.angles}")
            positions = determiner_position(tri)
            print("POSITION")
            print(positions)
            """
        """
        is_poteau_on_right = True
         ▲
        ▲ ▲
       ▲   ▲          □ POTEAU
      ▲     ▲

        """
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

        print("######")
        print([(center.pos_x, center.pos_y) for center in valid_centers])

        for tri in tri_list:
            position = self.determiner_position(tri)
            print("++++++++++++++++++++++++++++++")
            if position[0] > 0 and position[1] > 0:
                print(position)
                self.send_position(position)
        # Trier la liste par angle modulo pi

# avoir un lillian qui fait un faux triangle de la bonne taille
# filtrer les amalgames trop gros

# -> Avoir que quelques triangles valides

# -> Voir où est le poteau : A gauche ou à droite du triangle et distances cohérentes -> N'avoir plus qun' triangle valide

# Idée : calculer le centre du triangle et voir si il tombe sur la table

        list_pts = []
        msg_obj = msg_out

        # for tri in trianglel.valid_triangles:
        #    print("Liste des tiangles valides")
        #    print("[ " + str(tri.distances) +
        #          "," + str(tri.angles) + "]")
        # print("++++++++++++++++++++++++++++++")
        #
        # for tri in trianglel.valid_triangles:
        #    print("Liste des positions valides")
        #    position = self.determiner_position(tri)
        #    if position[0] > 0 and position[1] > 0:
        #        print(position)
        #    self.send_position(position)
        # print("++++++++++++++++++++++++++++++")

        """
        for tri in trianglel.tri_list:
            print("Liste des tiangles")
            print("[ " + str(tri.distances) +
                  "," + str(tri.angles) + "]")
        print("++++++++++++++++++++++++++++++")
        """
        """
        for i in range(0, len(msg_out.ranges)):
            for obj in objl.list_obj:
                if abs(obj.relative_center.angle - i * msg_out.angle_increment) < 0.1 :
                    list_pts.append(obj.relative_center.distance)
                else:
                    list_pts.append(0.0)


        msg_obj.ranges = list_pts
        """
        """
        for i in range(0, len(objl.list_obj)):
            print("Liste des Objets")
            print("[ " + str(objl.list_obj[i].relative_center.distance) +
                  "," + str(objl.list_obj[i].relative_center.angle) + "]")
        print("++++++++++++++++++++++++++++++")
        """
        """
        for i in range(0, len(objl.list_obj)):
            print("Liste des poitns des objets")
            for pt in objl.list_obj[i].list_points:
                print("[ " + str(pt.angle) + "]")
            print("--------")
        print("++++++++++++++++++++++++++++++")
        """
        """
        for i in range(0, len(objl.list_obj)):
            j = 0
            for pt in objl.list_obj[i].list_points:
                j += 1
                print("[ " + str(pt.distance) + ", " + str(pt.angle) + "]")
            print("-------- ObJ de " + str(j) + "points")
        print("++++++++++++++++++++++++++++++")
        """

        self.publisher_.publish(msg_obj)
    #à partir de l'amalgame de la tour indentifier l'indice des balices
    """def determiner_indice_balise_from_tour(self,tri,tour):
        for i in range(0, len(tri.pt_list)):
            if((calculate_distance_xy(tri.pt_list[i].pos_x, tri.pt_list[i].pos_y,tour.pos_x,tour.pos_y)>1.5) and (calculate_distance_xy(tri.pt_list[i].pos_x, tri.pt_list[i].pos_y,tour.pos_x,tour.pos_y)<1.7)):
            #je veux que ce point soit d'indice j
                j=i
            if((calculate_distance_xy(tri.pt_list[(i+1)%3].pos_x, tri.pt_list[(i+1)%3].pos_y,tour.pos_x,tour.pos_y)>2.4) and (calculate_distance_xy(tri.pt_list[(i+1)%3].pos_x, tri.pt_list[(i+1)%3].pos_y,tour.pos_x,tour.pos_y)<2.6)):
            #je veux que ce point soit d'indice i
                i=i+1    
            if((calculate_distance_xy(tri.pt_list[(i+2)%3].pos_x, tri.pt_list[(i+2)%3].pos_y,tour.pos_x,tour.pos_y)>1.7) and (calculate_distance_xy(tri.pt_list[(i+2)%3].pos_x, tri.pt_list[(i+2)%3].pos_y,tour.pos_x,tour.pos_y)<1.9)):
            #je veux que ce point soit d'indice k    
                k=i+2
    # def determiner_position_from_pts(self, list_pts):
    """
    def determiner_position(self, tri):
        x = 0
        y = 0
        x2 = 0
        y2 = 0

        """orient = 0

        for i in range(0, 2):
            if abs(tri.distances[i]) > 1.75 and abs(tri.distances[i]) < 1.95:
                orient = i+1
        # Determines the sides of the shortest triangle and the 2 other sides
        i = (orient + 1) % 3
        j = (orient + 3) % 3
        k = (orient + 2) % 3
        """
        i=0
        j=0
        k=0
        

        # Tri des points par angle tri dans le sens inverse des aiguilles d'une montre
        # print(tri.pt_list)
        tri.pt_list = sorted(tri.pt_list, key=Point.get_angle)
        print("test",calculate_distance_xy(tri.pt_list[0].pos_x, tri.pt_list[0].pos_y, tri.pt_list[1].pos_x, tri.pt_list[1].pos_y))
        # print(tri.pt_list)
        if((calculate_distance_xy(tri.pt_list[0].pos_x, tri.pt_list[0].pos_y, tri.pt_list[1].pos_x, tri.pt_list[1].pos_y)>1.6) and (calculate_distance_xy(tri.pt_list[0].pos_x, tri.pt_list[0].pos_y, tri.pt_list[1].pos_x, tri.pt_list[1].pos_y)<1.9)):
            print("k")
            j=0
            i=1
            k=2
        if((calculate_distance_xy(tri.pt_list[1].pos_x, tri.pt_list[1].pos_y, tri.pt_list[2].pos_x, tri.pt_list[2].pos_y)>1.6) and (calculate_distance_xy(tri.pt_list[1].pos_x, tri.pt_list[1].pos_y, tri.pt_list[2].pos_x, tri.pt_list[2].pos_y)<1.9)):
            print("j")
            j=1
            i=2
            k=0
        if((calculate_distance_xy(tri.pt_list[2].pos_x, tri.pt_list[2].pos_y, tri.pt_list[0].pos_x, tri.pt_list[0].pos_y)>1.6) and (calculate_distance_xy(tri.pt_list[2].pos_x, tri.pt_list[2].pos_y, tri.pt_list[0].pos_x, tri.pt_list[0].pos_y)<1.9)):
            print("l")
            j=2
            i=0
            k=1
        else:
            j=0
            i=1
            k=2            

        
        #print(0, len(tri.pt_list))
        # detectiin du plus petit coté et calcul de position
        """for i in range(0, len(tri.pt_list)):
            for j in range(0, i):
                d = abs(calculate_distance_xy(
                    tri.pt_list[i].pos_x, tri.pt_list[i].pos_y, tri.pt_list[j].pos_y, tri.pt_list[j].pos_x))
                #print(i, j, d)
                if (d > 1.7) and (d < 1.9):
                    break
        if i == j:
            print("pas  trouvé!!")
            raise
        """    
        #print(i, j)
        print(tri.pt_list[i].distance, tri.pt_list[j].distance,tri.pt_list[k].distance)

        teta = math.pi/2 - get_beta(tri.pt_list[j], tri.pt_list[i])
        phi = math.pi/2 - \
            get_gamma(tri.pt_list[i], tri.pt_list[j],
                      get_beta(tri.pt_list[i], tri.pt_list[j]))
        x = math.cos(teta)*tri.pt_list[i].distance - 0.1
        x2 = math.cos(phi)*tri.pt_list[j].distance - 0.1
        y = math.sin(teta)*tri.pt_list[i].distance + 0.05
        y2 = 1.95 - math.sin(phi)*tri.pt_list[j].distance

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

        """
        x = math.sqrt(tri.pt_list[j].distance**2 -
                    ((tri.pt_list[i].distance**2 - tri.pt_list[j].distance**2) - 3.61) / -1.9)**2
        y = ((tri.pt_list[i].distance**2 -
            tri.pt_list[j].distance**2 - 3.61) / -1.9)**2
        """
        resx = (x + x2)/2
        resy = (y + y2)/2

        alpha = math.acos(resx/tri.pt_list[j].distance)
        if tri.pt_list[j].angle > alpha:
            angle_N = tri.pt_list[j].angle-alpha
        else:
            angle_N = 2*math.pi-(alpha-tri.pt_list[j].angle)

        gisement = 0.0
        return [x, y, math.pi/2 - angle_N]


def get_distance_pt(pt1, pt2):
    return math.sqrt(pt1.distance**2 + pt2.distance**2 - 2*pt1.distance*pt2.distance*math.cos(abs(pt1.angle-pt2.angle)))


def get_beta(pt1, pt2):
    print(pt1.distance)
    print(pt2.distance)
    print(abs((get_distance_pt(pt1, pt2)**2 + pt2.distance**2 -
          pt1.distance**2) / (2 * get_distance_pt(pt1, pt2) * pt2.distance)))
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
