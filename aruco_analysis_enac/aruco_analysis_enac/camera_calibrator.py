#https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
#https://learnopencv.com/camera-calibration-using-opencv/
#https://docs.opencv.org/3.4/d4/d94/tutorial_camera_calibration.html
#https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#https://navigation.ros.org/tutorials/docs/camera_calibration.html

from array import array
import curses
import numpy as np
import cv2
import os

import glob
import yaml

import rclpy
import rclpy.node as node
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterDescriptor

def calib_path_parameter(node):
    #get current file path for calibration pictures at the parent and then subfolder called calibration
    
    calibration_folder_path = ParameterDescriptor()
    calibration_folder_path.name = 'save_folder_path'
    calibration_folder_path.description = 'Path to folder where pictures will be saved'
    calibration_folder_path.additional_constraints = 'save_folder_path'
    node.declare_parameter('calib_path', 'calib_imgs')
    calibration_folder_path = node.get_parameter('calib_path').get_parameter_value().string_value
    
    #create folder if it does not exist 
    try:
        os.chdir(calibration_folder_path)
        #chdir to one parent directory above the current one
    except FileNotFoundError:
        try:
            os.chdir(os.path.dirname(calibration_folder_path))
            os.mkdir(calibration_folder_path)
            os.chdir(calibration_folder_path)
        except FileNotFoundError:
            raise Exception("Calibration folder path not found - you need to set calib_path that point to at least a valid path (it can create one child if needed)")

    return calibration_folder_path

def input_ros_parameter(node):
    #get ros parameter for which input method to use
    node.declare_parameter('use_console_input', 
        False,
        ParameterDescriptor(description='use console input if set to true (without console output) ' \
            'instead of inputs from ros topics (calibration_take_picture and generate_aruco file)'))

    return node.get_parameter('use_console_input').get_parameter_value().bool_value

class Calibrator(node.Node):
    def __init__(self, calib_file_override=None) -> None:
        super().__init__('camera_calibrator')
        self.bridge = CvBridge()
        self.picture_to_take = 0
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.0001)
        self.width = None
        self.height = None
        self.info_msg = None
        
        #ros parameter to get a path string to save pictures
        self.calibration_folder_path = calib_path_parameter(self) if calib_file_override == None else calib_file_override

        self.use_console_input = input_ros_parameter(self)
        #ros parameter description for use_console_input
        self.info_sub = self.create_subscription(CameraInfo,
            'camera_info', #'/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)

        self.create_subscription(Image, '/image_raw',  #'/camera/image_raw',
            self.image_callback, qos_profile_sensor_data)

        self.downscale_img_pub = self.create_publisher(Image, '/image_calibration', qos_profile_sensor_data)
        
        #TODO : convert to services
        self.create_subscription(Bool, '/calibration_take_picture', self.take_picture, 10)

        self.create_subscription(Bool, '/generate_calibration_file', self.generate_calibration_file, 10)

    def info_callback(self, infos):
        """ get camera info and set width and height"""
        self.height = infos.height
        self.width =  infos.width
        self.info_msg = infos
        
        pass

    def image_callback(self, img_msg):
        """take picture and save it to the calibration folder if picture_to_take is not 0
        Args:
            img_msg (sensor_msgs.msg.Image): image message from camera
        """

        if self.info_msg == None:
            self.get_logger().info('No camera info received yet')
            return

        #convert image to cv2 and save it in a directory with the image nanosecond timestamp
        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
            desired_encoding='mono8')
        if self.picture_to_take >= 1:
            self.picture_to_take -= 1
            cv2.imwrite(str(self.get_clock().now().nanoseconds) + '.png', cv_image)

        #downscale image and publish it (for debug through wifi from raspberry pi)
        self.publish_calibration_picture (img_msg.header, cv_image, resize_height=144)

    def take_picture(self, bool_msg):
        print(bool_msg.data)
        if bool_msg.data == True:
            self.picture_to_take += 1

    def generate_calibration_file(self, bool_generate_msg):
        distorsion_model = 'fisheye' if bool_generate_msg.data else 'plum_bob' #true or false
        height = 9
        width = 7

        self.get_logger().info(f'generate_calibration_file with distorsion model {distorsion_model}')
        #TODO : wtf is used objp? 
        objp = np.zeros((height*width,3), np.float32)
        objp[:,:2] = np.mgrid[0:width,0:height].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        if distorsion_model == 'fisheye':
            #TODO : Use img_chessboard_extration before using generate_fisheye_params
            #TODO : remove last_img & frame_size
            rms, matrix_camera, dist, rvecs, tvecs, frame_size, img = self.generate_fisheye_params(objpoints, imgpoints)
        else: #distorsion_model == 'plum_bob'

            frame_size = None
            images = glob.glob(self.calibration_folder_path + '/*.png')
            for fname in images:
                img = cv2.imread(fname)
                frame_size = self.img_chessboard_extraction(img, objpoints, imgpoints)

            rms, matrix_camera, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

        proj_matrix = self.get_proj_matrix(matrix_camera, rvecs, tvecs)
        print("-- RMS for this calibration batch --")
        print(rms)
        if self.height == None: #no camera plugged
            self.height = frame_size[1]
        if self.width == None:
            self.width = frame_size[0]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix_camera, dist, (self.width,self.height), 1, (self.width,self.height))

        DIM = (self.width, self.height)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(matrix_camera, dist, np.eye(3), newcameramtx, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imwrite('calibresult.png', undistorted_img)

        self.write_yaml(
            self.generate_dict_camera_info(distorsion_model, matrix_camera, dist, proj_matrix))

    def get_proj_matrix(self, mtx, rvecs, tvecs):
        R = cv2.Rodrigues(rvecs[0])[0]
        t = tvecs[0]
        Rt = np.concatenate([R,t], axis=-1) # [R|t]
        P = np.matmul(mtx,Rt) # A[R|t]
        return P

    def img_chessboard_extraction(self, img, objpoints: list(), imgpoints: list()):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width,height), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            #cv2.drawChessboardCorners(img, (7,9, corners2, ret)
            #cv.imshow('img', img)
        frameSize = gray.shape[::-1]
        return frameSize
    def generate_fisheye_params(self, objpoints, imgpoints):
        """
        https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
        """
        # Checkboard dimensions
        CHECKERBOARD = (7,9)
        subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
        objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        np.expand_dims(np.asarray(objpoints), -2)
        ### read images and for each image:
        images = glob.glob(self.calibration_folder_path + '/*.png')
        N_imm = 0
        frame_size = None
        img = None #TODO : remove
        for fname in images:
            img = cv2.imread(fname)
            img_shape = img.shape[:2]

            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                imgpoints.append(corners2)
                N_imm += 1
                frame_size = gray.shape[::-1]
        #check if folder empty of picture or not (avoid assertion 215 error on calibrate)
        #if (len(objpoints) or len(imgpoints) or not (len(objpoints) == len(imgpoints))):
        #    self.get_logger().error("calibration folder is empty - can't calibrate")
        #    return
        # calculate K & D
        K = np.zeros((3, 3)) #matrix_camera
        D = np.zeros((4, 1)) #distorsion_coeff
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_imm)]
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints,
            imgpoints,
            frame_size,
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        
        return rms, K, D, rvecs, tvecs, frame_size, img

    """
    def generate_calibration_file_2(self):
        CHECKERBOARD = (7,7)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        #TODO : find images from path

        for filename in os.listdir(self.calibration_folder_path):
            img = cv2.imread(os.path.join(self.calibration_folder_path + filename))
            if img == None:
                self.get_logger().error(f'Could not read image {filename}')
                continue
            img_shape = img.shape[:2]
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                imgpoints.append(corners)
    """    

    def publish_calibration_picture(self, header, cv2img, resize_height=144):
        #draw markers
        ret = 1
        cv2.drawChessboardCorners(cv2img, (9, 7), None, ret)

        #downscale
        resize_width = int(resize_height * 16/9)
        resized = cv2.resize(cv2img, (resize_width, resize_height), interpolation = cv2.INTER_AREA)
        
        #reconvert to ros format along camera info
        img_ros = self.bridge.cv2_to_imgmsg(resized, encoding="8UC1")
        img_ros.header = header
        self.downscale_img_pub.publish(img_ros)
    
    
    def write_yaml(self, dict_file):
        with open(f'{self.calibration_folder_path}/calib_file.yaml', 'w') as f:
            yaml.dump(dict_file, f, sort_keys=False)

    def generate_dict_camera_info(self, distorsion_model, matrix_camera, distorsion, proj_matrix):
        """generate dict for camera info to make a yaml file

        Args:
            distorsion_model (_type_): _description_
            matrix_camera (_type_): _description_
            distorsion (_type_): _description_
            proj_matrix (_type_): _description_

        Returns:
            _type_: _description_
        """
        #TODO : have unique camera name

        #preparing data
        yaml_matrix_camera = matrix_camera.flatten().tolist()
        yaml_distotion_coeff = distorsion.flatten().tolist()
        yaml_rectification_matrix = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        yaml_projection_matrix = proj_matrix.flatten().tolist()
        column_dist_coeff = 5 if distorsion_model == 'plum_bob' else 4 #if fisheye

        #writing yaml file
        dict_file = {}
        dict_file['image_width'] = self.width
        dict_file['image_height'] = self.height
        dict_file['camera_name'] = 'camera_enac'
        dict_file['camera_matrix'] = {'rows': 3, 'cols': 3, 'data':yaml_matrix_camera} #TODO : unpack matrix_camera
        dict_file['distorsion_model'] = distorsion_model
        dict_file['distortion_coefficients'] = {'rows': 1, 'cols': column_dist_coeff, 'data':yaml_distotion_coeff}
        dict_file['rectification_matrix'] = {'rows': 3, 'cols': 3, 'data':yaml_rectification_matrix}
        dict_file['projection_matrix']  = {'rows': 3, 'cols': 4, 'data':yaml_projection_matrix}
        return dict_file


def main():
    rclpy.init()
    node = Calibrator()
    if node.use_console_input:
        while True:
            stdscr = curses.initscr()
            #curses.noecho() # Don't echo key presses
            stdscr.nodelay(1) # set getch() non-blocking

            stdscr.addstr(0,0,"Press \"p\" to take picture, \"g\" to generate calibration file, \"q\" to quit")
            try:
                c = stdscr.getch() #get character from stdin
                if c == ord('p'):
                    node.take_picture()
                elif c == ord('g'): 
                    node.generate_calibration_file()
                elif c == ord('q'): break
                rclpy.spin_once(node)

            finally:
                curses.endwin()
    else:
        rclpy.spin(node)
            
    node.destroy_node()
    rclpy.shutdown()
        
"""
        def is_good_sample(self, params, corners, ids, last_frame_corners, last_frame_ids):
        
        Returns true if the checkerboard detection described by params should be added to the database.
        
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a-b) for (a,b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        #print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        if d <= 0.2:
            return False

        if self.max_chessboard_speed > 0:
            if not self.is_slow_moving(corners, ids, last_frame_corners, last_frame_ids):
                return False

        # All tests passed, image should be good for calibration
        return True

    _param_names = ["X", "Y", "Size", "Skew"]

    def compute_goodenough(self):
        if not self.db:
            return None

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = lmin(min_params, params)
            max_params = lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.param_ranges)]
        # If we have lots of samples, allow calibration even if not all parameters are green
        # TODO Awkward that we update self.goodenough instead of returning it
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])

        return list(zip(self._param_names, min_params, max_params, progress))


        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
                    for fname in images:
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
            img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners

        images = glob.glob('*.jpg')
"""

if __name__ == "__main__":
    main()

    """
     if not fisheye:
            rms, matrix_camera, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
        else:
            rms, matrix_camera, dist, rvecs, tvecs = self.generate_fisheye_params(objpoints, imgpoints, frameSize)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            if distorsion_model == 'plum_bob':
                ret, corners = cv2.findChessboardCorners(gray, (width,height), None)
            elif distorsion_model == 'fisheye':
                ret, corners = cv2.findChessboardCorners(gray, (width,height), cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                #cv2.drawChessboardCorners(img, (7,9, corners2, ret)
                #cv.imshow('img', img)
            frameSize = gray.shape[::-1]
        if distorsion_model == 'plum_bob':
            rms, matrix_camera, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)
        elif distorsion_model == 'fisheye':
            objp = np.zeros((1, width*height, 3), np.float32)
            objp[0,:,:2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
            N_OK = len(objpoints)
            K = np.zeros((3, 3))
            D = np.zeros((4, 1))
            rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
            tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
            np.expand_dims(np.asarray(objpoints), -2) #https://github.com/opencv/opencv/issues/9150
            rms, matrix_camera, dist, rvecs, tvecs = \
                cv2.fisheye.calibrate(
                    objpoints,
                    imgpoints,
                    frameSize,
                    K,
                    D,
                    rvecs,
                    tvecs,
                    calibration_flags,
                    (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
                )
                """
