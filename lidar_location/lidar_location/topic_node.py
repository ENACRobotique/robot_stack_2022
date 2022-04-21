from lidar_location.Triangulation import Triangulation
from lidar_location.Amalgame_list import Amalgame_list
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
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

    def save_valid_to_file(self, tri, file):
        file_object = open(file, 'a')
        output = str(tri.angles[0]) + ";" + str(tri.angles[1]) + ";" + str(tri.angles[2]) + ";" + str(
            tri.distances[0]) + ";" + str(tri.distances[1]) + ";" + str(tri.distances[2])
        file_object.write(output)
        # print(output)
        file_object.close()

    def listener_callback(self, msg):
        # self.get_logger().info(msg.angle_max)
        msg_out = self.generate_filtered_message(msg, self.filter_out(msg))
        #msg_out = self.generate_filtered_message(msg, self.filter_out(self.generate_fake(msg)))
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
        tri.distances[0] > 1.7 and tri.distances[0] < 2 and tri.distances[
            1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 3 and tri.distances[2] < 3.4
        or
        tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[
            1] > 1.7 and tri.distances[1] < 2 and tri.distances[2] > 3 and tri.distances[2] < 3.4
        or
        tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[
            1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 1.7 and tri.distances[2] < 2

        tri.distances[0] > 1.7 and tri.distances[0] < 2 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[
            1] > 1.7 and tri.distances[1] < 2 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 1.7 and tri.distances[2] < 2
        """

        """
        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9
        or
        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3
        or
        tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3

        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 or tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[
            2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 or tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3
        
        dist1_low = 3.3
            dist1_high = 3.35
            angle1_low = 1.2
            angle1_high = 1.3
            dist2_low = 1.7
            dist2_high = 1.9
            angle2_low = 0.55
            angle2_high = 0.7
            if ((tri.distances[0] > dist2_low and tri.distances[0] < dist2_high) and (tri.distances[1] > dist1_low and tri.distances[1] < dist1_high) and (tri.distances[2] > dist1_low and tri.distances[2] < dist1_high)) or ((tri.distances[0] > dist1_low and tri.distances[0] < dist1_high) and (tri.distances[1] > dist2_low and tri.distances[1] < dist2_high) and (tri.distances[2] > dist1_low and tri.distances[2] < dist1_high)) or ((tri.distances[0] > dist1_low and tri.distances[0] < dist1_high) and (tri.distances[1] > dist1_low and tri.distances[1] < dist1_high) and (tri.distances[2] > dist2_low and tri.distances[2] < dist2_high)) and ((tri.angles[0] > angle1_low and tri.angles[0] < angle1_high) and (tri.angles[1] > angle1_low and tri.angles[1] < angle1_high) and (tri.angles[2] > angle2_low and tri.angles[2] < angle2_high)) or ((tri.angles[0] > angle1_low and tri.angles[0] < angle1_high) and (tri.angles[2] > angle2_low and tri.angles[2] < angle2_high) and (tri.angles[1] > angle1_low and tri.angles[1] < angle1_high)) or ((tri.angles[2] > angle2_low and tri.angles[2] < angle2_high) and (tri.angles[0] > angle1_low and tri.angles[0] < angle1_high) and (tri.angles[1] > angle1_low and tri.angles[1] < angle1_high)):

        
        """
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
        objl = Amalgame_list(msg_out)
        # self.get_logger().info("Une list dobj:")
        trianglel = Triangulation(msg_out)
        list_pts = []
        msg_obj = msg_out

        for tri in trianglel.valid_triangles:
            print("Liste des tiangles valides")
            print("[ " + str(tri.distances) +
                  "," + str(tri.angles) + "]")
        print("++++++++++++++++++++++++++++++")

        for tri in trianglel.valid_triangles:
            print("Liste des positions valides")
            position = self.determiner_position(tri)
            if position[0] > 0 and position[1] > 0:
                print(position)
            self.send_position(position)
        print("++++++++++++++++++++++++++++++")

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

    def determiner_position(self, tri):
        x = 0
        y = 0
        x2 = 0
        y2 = 0

        orient = 0

        for i in range(0, 2):
            if abs(tri.distances[i]) > 1.75 and abs(tri.distances[i]) < 1.95:
                orient = i+1
        # Determines the sides of the shortest triangle and the 2 other sides
        c1 = (orient + 1) % 3
        c2 = (orient + 3) % 3
        c3 = (orient + 2) % 3

        # voir ou qu'il pense qu'elles sont les balises
        msg_bal_1 = TransformStamped()
        msg_bal_1.header.frame_id = "laser"
        msg_bal_1.child_frame_id = "balise1"
        msg_bal_1.transform.translation.x = tri.pt_list[c1].distance * math.cos(
            tri.pt_list[c1].angle)
        msg_bal_1.transform.translation.y = tri.pt_list[c1].distance * math.sin(
            tri.pt_list[c1].angle)
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
        msg_bal_2.transform.translation.x = tri.pt_list[c2].distance * math.cos(
            tri.pt_list[c2].angle)
        msg_bal_2.transform.translation.y = tri.pt_list[c2].distance * math.sin(
            tri.pt_list[c2].angle)
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
        msg_bal_3.transform.translation.x = tri.pt_list[c3].distance * math.cos(
            tri.pt_list[c3].angle)
        msg_bal_3.transform.translation.y = tri.pt_list[c3].distance * math.sin(
            tri.pt_list[c3].angle)
        msg_bal_3.transform.translation.z = 0.0
        [qx3, qy3, qz3, qw3] = quaternion_from_euler(
            0, 0, 0)

        msg_bal_3.transform.rotation.x = qx3
        msg_bal_3.transform.rotation.y = qy3
        msg_bal_3.transform.rotation.z = qz3
        msg_bal_3.transform.rotation.w = qw3
        self.publisher_map.publish(msg_bal_3)

        teta = math.pi/2 - get_beta(tri.pt_list[c2], tri.pt_list[c1])
        phi = math.pi/2 - \
            get_gamma(tri.pt_list[c2], tri.pt_list[c1],
                      get_beta(tri.pt_list[c2], tri.pt_list[c1]))
        x = math.cos(teta)*tri.pt_list[c1].distance - 0.1
        x2 = math.cos(phi)*tri.pt_list[c2].distance - 0.1
        y = math.sin(teta)*tri.pt_list[c1].distance + 0.05
        y2 = 1.95 - math.sin(phi)*tri.pt_list[c2].distance

        """
        x = math.sqrt(tri.pt_list[c2].distance**2 -
                    ((tri.pt_list[c1].distance**2 - tri.pt_list[c2].distance**2) - 3.61) / -1.9)**2
        y = ((tri.pt_list[c1].distance**2 -
            tri.pt_list[c2].distance**2 - 3.61) / -1.9)**2
        """
        resx = (x + x2)/2
        resy = (y + y2)/2

        alpha = math.acos(resx/tri.pt_list[c2].distance)
        if tri.pt_list[c2].angle > alpha:
            angle_N = tri.pt_list[c2].angle-alpha
        else:
            angle_N = 2*math.pi-(alpha-tri.pt_list[c2].angle)

        gisement = 0.0
        return [x, y, math.pi/2 - angle_N]


def get_distance_pt(pt1, pt2):
    return math.sqrt(pt1.distance**2 + pt2.distance**2 - 2*pt1.distance*pt2.distance*math.cos(abs(pt1.angle-pt2.angle)))


def get_beta(pt1, pt2):
    # print(pt1.distance)
    # print(pt2.distance)
    #print(abs((get_distance_pt(pt1, pt2)**2 + pt2.distance**2 - pt1.distance**2) / (2 * get_distance_pt(pt1, pt2) * pt2.distance)))
    return math.acos(abs((get_distance_pt(pt1, pt2)**2 + pt2.distance**2 - pt1.distance**2) / (2 * get_distance_pt(pt1, pt2) * pt2.distance)))


def get_gamma(pt1, pt2, beta):
    return math.pi - beta - abs(pt2.angle - pt1.angle)


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
