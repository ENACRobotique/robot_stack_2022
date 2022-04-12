from lidar_location.Triangulation import Triangulation
from lidar_location.Amalgame_list import Amalgame_list
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math


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
        #self.publish_list_obj = self.create_publisher(LaserScan, 'list_obj', 10)

    def generate_filtered_message(self, message, filtered_data):
        out = message
        for i in range(0, len(filtered_data)):
            out.ranges[i] = filtered_data[i]
        return out

    def filter_out(self, message):
        out = []
        for i in range(0, len(message.ranges)):
            if message.ranges[i] is not None:
                if message.ranges[i] > 3.6:
                    out.append(0)
                else:
                    out.append(message.ranges[i])
            else:
                out.append(message.ranges[i])
        return out

    def echo(self, message):
        return message

    def listener_callback(self, msg):
        # self.get_logger().info(msg.angle_max)
        msg_out = self.generate_filtered_message(msg, self.filter_out(msg))
        #msg_out = msg
        # self.get_logger().info(msg_out.angle_max)
        #triangulation = Amalgame_list(msg_out)
        triangulation = Triangulation(msg_out)
        #self.get_logger().info("-------------")
        # for obj in triangulation.list_obj:
        #    self.get_logger().info(
        #        f"{obj.relative_center}")
        # self.publisher_.publish(msg_out)

        """
        tri.distances[0] > 1.7 and tri.distances[0] < 2 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 3 and tri.distances[2] < 3.4
        or
        tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 1.7 and tri.distances[1] < 2 and tri.distances[2] > 3 and tri.distances[2] < 3.4
        or
        tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 1.7 and tri.distances[2] < 2

        tri.distances[0] > 1.7 and tri.distances[0] < 2 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 1.7 and tri.distances[1] < 2 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 1.7 and tri.distances[2] < 2
        """

        """
        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9
        or
        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3
        or
        tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3

        tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 or tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 or tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3
        """
        
        for tri in triangulation.tri_list:
            #self.get_logger().info("------------------------------------------------------")
            if (tri.distances[0] > 1.7 and tri.distances[0] < 2 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 1.7 and tri.distances[1] < 2 and tri.distances[2] > 3 and tri.distances[2] < 3.4 or tri.distances[0] > 3 and tri.distances[0] < 3.4 and tri.distances[1] > 3 and tri.distances[1] < 3.4 and tri.distances[2] > 1.7 and tri.distances[2] < 2) and (tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 or tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3 or tri.angles[2] > 0.6 and tri.angles[2] < 0.9 and tri.angles[0] > 1.1 and tri.angles[0] < 1.4 and tri.angles[1] > 1.1 and tri.angles[1] < 1.3):
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
                
        
        
        objl = Amalgame_list(msg_out)
        #self.get_logger().info("Une list dobj:")
        list_pts = []
        msg_obj = msg_out
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
            print("[ " + str(objl.list_obj[i].relative_center.distance) + "," + str(objl.list_obj[i].relative_center.angle) + "]")
        print("++++++++++++++++++++++++++++++")
        """

        """
        for i in range(0, len(objl.list_obj)):
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
                print("[ " + str(pt.angle) + "]")
            print("-------- ObJ de " + str(j) + "points")
        print("++++++++++++++++++++++++++++++")
        """
        
            #self.get_logger().info("Un Amalgame:")
            #x = obj.relative_center.distance * math.cos(obj.relative_center.angle)
            #y = obj.relative_center.distance * math.sin(obj.relative_center.angle)
            #self.get_logger().info(
            #    f"{obj.relative_center.distance}")
            #self.get_logger().info(
            #    f"{obj.relative_center.angle}")
        

        self.publisher_.publish(msg_obj)

def get_distance_pt(pt1, pt2):
        return math.sqrt(pt2.distance**2 + pt1.distance**2 - 2 * pt1.distance * pt2.distance * math.cos(pt1.angle - pt2.angle))

def get_beta(pt1, pt2):
        return abs(math.acos((get_distance_pt(pt1, pt2)**2 + pt1.distance**2 - pt2.distance**2) / (2 * get_distance_pt(pt1, pt2) * pt1.distance)))%math.pi

def get_gamma(pt1, pt2, beta):
        return abs(math.pi - beta - abs(pt2.angle - pt1.angle))%math.pi

def determiner_position(tri):
    
    
    
    
    
    
    
    
    if abs(tri.distances[0]) > 1.7 and abs(tri.distances[0]) < 2 :
        #beta1 = get_beta(tri.pt_list[0], tri.pt_list[1])
        #gamma1 = get_gamma(tri.pt_list[0], tri.pt_list[1], beta1)
        #theta = math.pi/2 - beta1 
        #phi = math.pi/2 - gamma1
        #x = 2 / (math.tan(theta) + math.tan(phi))
        #y = 2 * math.tan(theta) / (math.tan(theta) + math.tan(phi))
        teta= math.pi/2 - get_beta(tri.pt_list[0], tri.pt_list[1])
        x= math.cos(teta)*tri.pt_list[0].distance
        y= math.sin(teta)*tri.pt_list[0].distance
        
        
        #y = (tri.pt_list[0].distance**2 - tri.pt_list[1].distance**2 -4)/-2
        #x = (math.sqrt(tri.pt_list[1].distance**2)-y**2)
        return [x,y]

    if abs(tri.distances[1]) > 1.7 and abs(tri.distances[1]) < 2 :
        #beta1 = get_beta(tri.pt_list[1], tri.pt_list[2])
        #gamma1 = get_gamma(tri.pt_list[1], tri.pt_list[2], beta1)
        #theta = math.pi/2 - beta1 
        #phi = math.pi/2 - gamma1
        #x = 2 / (math.tan(theta) + math.tan(phi))
        #y = 2 * math.tan(theta) / (math.tan(theta) + math.tan(phi))
        #y = (tri.pt_list[1].distance**2 - tri.pt_list[2].distance**2 -4)/-2
        #x = (math.sqrt(tri.pt_list[2].distance**2)-y**2)
        teta= math.pi/2 - get_beta(tri.pt_list[1], tri.pt_list[2])
        x= math.cos(teta)*tri.pt_list[1].distance
        y= math.sin(teta)*tri.pt_list[1].distance
        return [x,y]

    if abs(tri.distances[2]) > 1.7 and abs(tri.distances[2]) < 2 :
        #beta1 = get_beta(tri.pt_list[2], tri.pt_list[0])
        #gamma1 = get_gamma(tri.pt_list[2], tri.pt_list[0], beta1)
        #theta = math.pi/2 - beta1 
        #phi = math.pi/2 - gamma1
        #x = 2 / (math.tan(theta) + math.tan(phi))
        #y = 2 * math.tan(theta) / (math.tan(theta) + math.tan(phi))
        #y = (tri.pt_list[2].distance**2 - tri.pt_list[0].distance**2 -4)/-2
        #x = (math.sqrt(tri.pt_list[0].distance**2)-y**2)
        teta= math.pi/2 - get_beta(tri.pt_list[2], tri.pt_list[0])
        x= math.cos(teta)*tri.pt_list[2].distance
        y= math.sin(teta)*tri.pt_list[2].distance
        
        return [x,y]

        



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
