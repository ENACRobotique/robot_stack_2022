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
        self.detectorSettings = None #format - [[[ids], size, resolution], ...]

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
        #self.detectorService = self.create_service(customArucoDetector, 'custom_aruco_detector', self.custom_detector_cb)

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
        
        # pose estimation
        if len(ids) >= 1:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,0.05, self.intrinsic_mat, self.distortion)
            self.get_logger().debug(f"pose estimation for arucos : \n rvecs : {rvecs} \n tvecs : {tvecs}")
            msg_ids = []
            for id in ids:
                msg_ids.append(int(id))
            self.publish_markers(img_msg.header, msg_ids, rvecs, tvecs)
            #self.get_logger().info(str((self.get_clock().now()-timeTaken)))

            #generate downscaled picture for debug purposes
            if self.debug_mode:
                self.publish_img(img_msg.header, corners, ids, cv_image, 144)
                # self.get_logger().debug(f"aruco_detected : {corners} id, rejected)

            #self.get_logger().info(str((self.get_clock().now()-timeTaken)))
        else:
            self.get_logger().info("ids not detected")

    def custom_detector_cb(self, detector_settings_srv):
        if self.detectorSettings == None:
            self.detectorSettings = []
        self.detectorSettings.append([detector_settings_srv.ids, detector_settings_srv.size, detector_settings_srv.resolution])

    def publish_markers(self, header, ids, rvecs, tvecs):
        pose_msg = FidPoses()
        pose_msg.header = header.header
        pose_msg.marker_ids = ids
        pose_msg.rvecs = []
        pose_msg.tvecs = []
        for rvec in rvecs.tolist():
            pose_msg.rvecs.append(Vector3(x=rvec[0][0], y=rvec[0][1], z=rvec[0][2]))
        for tvec in tvecs.tolist():
            pose_msg.tvecs.append(Vector3(x=tvec[0][0], y=tvec[0][1], z=tvec[0][2]))
        print(header.header.stamp)
        self.markers_pose_pub.publish(pose_msg)

    def publish_img(self, header, cv2_img, corners, ids, resize_height=144):
        img_with_markers = cv2.aruco.drawDetectedMarkers(cv2_img, corners, ids)
        original_width = img_with_markers.shape[1]
        resized_height = resize_height
        resized = cv2.resize(img_with_markers, [original_width, resized_height], interpolation = cv2.INTER_AREA)
        img_ros = self.bridge.cv2_to_imgmsg(resized, encoding="8UC1")
        img_ros.header = header
        self.markers_image_pub.publish(img_ros)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()