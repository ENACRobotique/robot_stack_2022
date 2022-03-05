import unittest
import pytest
import rclpy

import aruco_analysis_enac.camera_calibrator as cam_cal

class TestCameraCalibrator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.Calibrator()
        self.node.set_parameter('calib_path', 
            '/enac_ws/src/robot_stack_2022/aruco_analysis_enac/calibration')

    def tearDown(self):
        self.node.destroy_node()
    #testNode = cam_cal.Calibrator()

    def test_instantiation_node(self):
        # Test if the node is instantiated
        assert self.node != None

    def test_dict_for_yaml(self):
        distorsion_model = 'plumb_bob'
        matrix_camera = []
        distorsion_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
        dict = self.node.generate_dict_camera_info(distorsion_model, matrix_camera, distorsion_coeff)
        assert dict['image_height'] == 2050
        assert dict['camera_name'] == 'camera_enac'
        assert dict['camera_matrix'] == [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        assert dict['distortion_model'] == 'plumb_bob'
        assert dict['distortion_coefficients'] == [0.0, 0.0, 0.0, 0.0, 0.0]
        assert dict['rectification_matrix'] == [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        assert dict['projection_matrix'] == [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        

if __name__ == '__main__':
    unittest.main()
