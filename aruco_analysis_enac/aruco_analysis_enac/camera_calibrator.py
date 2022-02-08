#https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
#https://learnopencv.com/camera-calibration-using-opencv/
#https://docs.opencv.org/3.4/d4/d94/tutorial_camera_calibration.html
#https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#https://navigation.ros.org/tutorials/docs/camera_calibration.html
"""

Fonctionalités nécessaire :
Prendre un input pour prendre une photo
Indiquer le "pourcentage de variation"
CAlculer
générer le yaml standardisé
"""
from click import pass_context
import numpy as np
import cv2
import glob

import rclpy
import rclpy.node as node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

class Calibrator(node.Node):
    def __init__(self) -> None:
        super().__init__()
        self.bridge = CvBridge()
        self.cv2_pictures_taken = []
        self.picture_to_take = 0
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        self.info_sub = self.create_subscription(CameraInfo,
            'camera_info', #'/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data)

        self.create_subscription(Image, '/image_raw',  #'/camera/image_raw',
            self.image_callback, qos_profile_sensor_data)

        self.create_publisher(Image, '/image_calibration')

        self.create_subscrition(Bool, '/calibration_take_picture', self.take_picture, 10)

        def image_callback(self, img_msg):
            if self.info_msg == None:
                return

            cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                desired_encoding='mono8')


cv2.destroyAllWindows()


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

"""
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        """
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
        def generate_calibration_file(self):
            pass

        def publish_calibration_picture(self, cv2img):
            #rosimg = cv2img.
            pass

def main():
    rclpy.init()
    node = Calibrator()
    while True:
        menu = input(" input 1 to take picture, input 2 to generate yaml calibration file")
        if int(menu) == 1:
            node.take_picture()
        elif int(menu) == 2:
            node.generate_calibration_file()
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
        
