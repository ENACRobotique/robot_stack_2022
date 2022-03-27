import cv2
import numpy as np
import signal
from contextlib import contextmanager
import aruco_analysis_enac.settings as settings

import rclpy
import rclpy.node as node
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from interfaces_enac.msg import _fiducials_poses

FidPoses = _fiducials_poses.FiducialsPoses
class ArucoNode(node.Node):
    def __init__(self):
        super().__init__('detect_aruco')
        self.declare_parameter('debug_mode', True) #scan all arucos every time and publish a downscaled picture for debug
        settings_description = ParameterDescriptor(description='aruco settings subset from settings.py (by int)')
        self.declare_parameter('subset', 1, settings_description)
        self.declare_parameter('is_fish_cam', False)

        self.bridge = CvBridge()
        self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.info_msg = None #set on /camera_info/ topic
        self.detectorSettings = None #format - [[[ids], size, resolution], ...]
        self.frame_counter = 0

        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.get_logger().info(f"debug mode : {self.debug_mode}")
        self.marker_settings = settings.get_markers(self.get_parameter('subset').get_parameter_value().integer_value)
        self.is_fisheye = self.get_parameter('is_fish_cam').get_parameter_value().bool_value
        if self.is_fisheye:
            self.map1, self.map2 = None, None #used for undistortion

        self.info_sub = self.create_subscription(CameraInfo,
            'camera_info', #'/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)

        self.create_subscription(Image, '/image_raw',  #'/camera/image_raw',
            self.image_callback, 10)

        self.markers_pose_pub = self.create_publisher(FidPoses, 'aruco_poses', 10)
        if self.debug_mode:
            self.markers_image_pub = self.create_publisher(Image, 'aruco_pictures', qos_profile_sensor_data)
        #self.detectorService = self.create_service(customArucoDetector, 'custom_aruco_detector', self.custom_detector_cb)

    def info_callback(self, info_msg):
        """ """
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        if self.is_fisheye:
            DIM = (self.info_msg.width, self.info_msg.height)
            newcameramtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.intrinsic_mat, self.distortion, DIM, np.eye(3), balance=1)


            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(self.intrinsic_mat, self.distortion, np.eye(3), newcameramtx, DIM, cv2.CV_16SC2)

        self.get_logger().debug('info from camera has been added : ')
        self.get_logger().debug(info_msg)
        # Assume that camera parameters will remain the same
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        """ receive image and send detected markers pose and taking into account size of markers """
        timeTaken = self.get_clock().now()
        if self.info_msg == None:
            return
        self.frame_counter += 1
        print(self.is_fisheye)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg)
        if self.is_fisheye: #undistort the image in case of fisheye camera (i'm trying to fix errors from fisheye)
            cv_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            pass

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.dict)
        # pose estimation
        if ids is not None and len(ids) >= 1:
            mvt_flag = settings.get_movement_flag(self.frame_counter) if not self.debug_mode else settings.Movement.ALL
            corners_by_size = self.marker_settings.regroup_corners_by_size(corners, ids, self.get_logger(), mvt_flag)
            ids_sorted, rvecs, tvecs = [], [], []
            for size in corners_by_size:
                #rvecs and tvecs from current treated size
                rvecs_cur_size, tvecs_cur_size, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners_by_size[size][0], size, self.intrinsic_mat, self.distortion)
                rvecs.extend(rvecs_cur_size)
                tvecs.extend(tvecs_cur_size)
                ids_sorted.extend(corners_by_size[size][1])
                self.get_logger().debug(f"pose estimation for arucos : \n rvecs : {rvecs} \n tvecs : {tvecs}")

            #print("3"+ str(self.get_clock().now() - timeTaken))

            self.publish_markers(img_msg.header, ids_sorted, rvecs, tvecs)
        if self.debug_mode and rvecs is not None and tvecs is not None: #prevent from publishing image without corners which would crash the node
            self.publish_img(img_msg.header, cv_image, corners, ids, rvecs, tvecs, 240)

        else:
            self.get_logger().info("ids not detected")


    def publish_markers(self, header, ids, rvecs, tvecs)->None:
        """ Publish markers pose in a FiducialsPoses message """
        pose_msg = FidPoses()
        pose_msg.header = header
        pose_msg.marker_ids = [int(id) for id in ids]
        pose_msg.rvecs = []
        pose_msg.tvecs = []
        for rvec in rvecs:
            pose_msg.rvecs.append(Vector3(x=rvec[0][0], y=rvec[0][1], z=rvec[0][2]))
        for tvec in tvecs:
            pose_msg.tvecs.append(Vector3(x=tvec[0][0], y=tvec[0][1], z=tvec[0][2]))
        self.get_logger().info(f"{header.stamp}")
        self.markers_pose_pub.publish(pose_msg)
    def publish_img(self, header, cv2_img, corners, ids, rvecs, tvecs, resize_height=144)->None:
        """ 
            Publish image with detected markers downscaled to a given height
        """
        if corners != None:
            img_with_markers = cv2.aruco.drawDetectedMarkers(cv2_img, corners, np.array(ids))

        #Adding timeout because function sometimes takes too long to finish
        with timeout(1):
            for i, sized_rvec in enumerate(rvecs):
                for y, rvec in enumerate(sized_rvec):
                    cv2.aruco.drawAxis(img_with_markers, self.intrinsic_mat, self.distortion, rvec, tvecs[i][y], 0.1)
        resize_width = int(resize_height * 16/9)
        resized = cv2.resize(img_with_markers, (resize_width, resize_height), interpolation = cv2.INTER_AREA)
        img_ros = self.bridge.cv2_to_imgmsg(resized, encoding="8UC3")
        img_ros.header = header
        self.markers_image_pub.publish(img_ros)

#https://www.jujens.eu/posts/en/2018/Jun/02/python-timeout-function/
@contextmanager
def timeout(time):
    # Register a function to raise a TimeoutError on the signal.
    signal.signal(signal.SIGALRM, raise_timeout)
    # Schedule the signal to be sent after ``time``.
    signal.alarm(time)

    try:
        yield
    except TimeoutError:
        print(f" draw axis has timeout : exceeded 1s ")
        pass
    finally:
        # Unregister the signal so it won't be triggered
        # if the timeout is not reached.
        signal.signal(signal.SIGALRM, signal.SIG_IGN)

def raise_timeout(signum, frame):
    raise TimeoutError



def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()