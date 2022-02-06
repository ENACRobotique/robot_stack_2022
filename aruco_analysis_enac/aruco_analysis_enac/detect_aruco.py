import cv2
import numpy as np

import rclpy
import rclpy.node as node
from cv_bridge import CvBridge

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from interfaces_enac.msg import _fiducials_poses

from aruco_analysis_enac.aruco_calculations import rotation

FidPoses = _fiducials_poses.FiducialsPoses

class ArucoNode(node.Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.bridge = CvBridge()
        self.info_msg = None #set on /camera_info/ topic

        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.get_logger().info(f"debug mode : {self.debug_mode}")
        self.info_sub = self.create_subscription(CameraInfo,
            'camera_info', #'/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)

        self.create_subscription(Image, '/image_raw',  #'/camera/image_raw',
            self.image_callback, qos_profile_sensor_data)

        self.markers_pose_pub = self.create_publisher(FidPoses, 'aruco_poses', qos_profile_sensor_data)
        if self.debug_mode:
            print("debug 2")
            self.markers_image_pub = self.create_publisher(Image, 'aruco_pictures', qos_profile_sensor_data)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().debug('info from camera has been added : ')
        self.get_logger().debug(info_msg)
        # Assume that camera parameters will remain the same
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        timeTaken = self.get_clock().now()
        if self.info_msg == None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
            desired_encoding='mono8')
        dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, dict)

        """"   
        rvecsCamera, tvecsCamera = [], []
        print(corners)
        try:
            for i in range(len(tvecs)):
                rvecsCamera.append(rotation(-rvecs[i][0], -rvecs[i][1], -rvecs[i][2], tvecs[i]))
                tvecsCamera.append([-tvecs[i][0], tvecs[i][1], tvecs[i][2]])
        except:
            print("exception 223233242")
        """    
        
        # pose estimation
        if len(ids) >= 1:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,0.07, self.intrinsic_mat, self.distortion)
            self.get_logger().debug(f"pose estimation for arucos : \n rvecs : {rvecs} \n tvecs : {tvecs}")
            msg_ids = []
            for id in ids:
                msg_ids.append(int(id))

            #self.get_logger().info("bbbbb")
            #self.get_logger().info(str((self.get_clock().now()-timeTaken)))

            pose_msg = FidPoses()
            pose_msg.header = img_msg.header
            pose_msg.marker_ids = msg_ids
            pose_msg.rvecs = []
            pose_msg.tvecs = []
            for rvec in rvecs.tolist():
                pose_msg.rvecs.append(Vector3(x=rvec[0][0], y=rvec[0][1], z=rvec[0][2]))
            for tvec in tvecs.tolist():
                pose_msg.tvecs.append(Vector3(x=tvec[0][0], y=tvec[0][1], z=tvec[0][2]))
            print(img_msg.header.stamp)
            self.markers_pose_pub.publish(pose_msg)
            print("test")

            if self.debug_mode:
                # self.get_logger().debug(f"aruco_detected : {corners} id, rejected)
                img_with_markers = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                original_width = img_with_markers.shape[1]
                resized_height = 144
                resized = cv2.resize(img_with_markers, [original_width, resized_height], interpolation = cv2.INTER_AREA)
                img_ros = self.bridge.cv2_to_imgmsg(resized, encoding="8UC1")
                img_ros.header = img_msg.header
                self.markers_image_pub.publish(img_ros)
            #self.get_logger().info(str((self.get_clock().now()-timeTaken)))
        else:
            self.get_logger().info("ids not detected")

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()