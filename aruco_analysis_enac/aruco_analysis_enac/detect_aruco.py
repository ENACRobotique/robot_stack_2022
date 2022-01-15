from cv2 import cv2
import numpy as np

import rclpy
import rclpy.node as node
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class ArucoNode(node.Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.bridge = CvBridge()

        self.info_sub = self.create_subscription(CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)
        print("bbb")
    
        self.create_subscription(Image, '/camera/image_raw',
            self.image_callback, qos_profile_sensor_data)

        self.markers_pub = self.create_publisher(Image, 'arucos', 10)
    
    def info_callback(self, info_msg):
        print("a")
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
            desired_encoding='mono8')

        #ajout d'axes
        dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, dict)
        print(corners)
        img_with_markers = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        img_ros = self.bridge.cv2_to_imgmsg(img_with_markers, encoding="8UC1")
        img_ros.header.frame_id="default_cam"
        print(img_ros.header)
        self.markers_pub.publish(img_ros)

        try:
            print("aaa")
            #poste estimation
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,0.07, self.intrinsic_mat, self.distortion)
            print(rvecs)
            print(tvecs)
            print(_)
        except:
            print("pb info cam")

                # parameters=arucoParams)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()