#TODO : to remove
import math
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


import aruco_analysis_enac.aruco_calculations as calc
from aruco_analysis_enac.aruco_storage import ArucosStorage
from interfaces_enac.msg import _fiducials_poses

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


FidPoses = _fiducials_poses.FiducialsPoses

class ArucoAnalysis(Node):

    def __init__(self, arucoStorage = None, name='aruco_analysis', fixedArucoRate=1.0, movingArucoRate=0.1):
        super().__init__(name)

        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        if arucoStorage:
            self.arucosStorage = arucoStorage
        else:
            self.arucosStorage = ArucosStorage(
                #[calc.Aruco(42, 0.15, 0.10, 0.0)],
                [calc.Aruco(36, 0.15, 0.10, 0.0)], 
                'map')

        self.aruco_poses = self.create_subscription(
            FidPoses,
            '/aruco_poses',
            self.__handle_arucos,
            qos_profile_sensor_data
        )

        self.tf_publisher = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        #TODO : faire une classe à part pour la gestion des publishers avec 2 rate différents
        #self.object_poses_publisher = self.create_publisher(Aruco, 'object_poses', 10)
        #self.create_timer(movingArucoRate, self.object_poses_publisher)

        if self.debug_mode:
            self.diagnostics = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                10
            )
    def __handle_arucos(self, aruco_poses):
        """Analysis is done in two steps
        first, we determine camera position, then we analyse the "free" aruco, the one we don't know their position on the table
        Args:
            aruco_poses ([type]): [description]
        """
        tm = TransformManager()
        now = aruco_poses.header.stamp
        aurcosIdsIndex = [] #not reference
        cameraPoseEnac = None
        for i, id in enumerate(aruco_poses.marker_ids):
            if id in self.arucosStorage.reference_ids:
                pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                print("rvec : ")
                print(aruco_poses.rvecs[i])
                MIDDLE = calc.Pose(1.50, 1.0, 0.0, (0,0,0)) #TODO : take from aruco storage
                cameraPoseEnac = calc.get_camera_position(MIDDLE, pose) #TODO : faire une fusion de données, là on se contente de prendre le dernier
                
                #tm.add_transform("camera",'aruco')

                #ax = tm.plot_frames_in("camera", s=0.1)
                #plt.slow()
                
                #calc.publish_pos_from_reference(self.tf_publisher, now, MIDDLE, 'origin', 'middle_map')
                calc.publish_pos_from_reference(self.tf_publisher, now, cameraPoseEnac, 'camera', 'aruco')

                self.get_logger().info(
                    f"according to reference {id}, camera is at {cameraPoseEnac}"
                )
            else:
                aurcosIdsIndex.append(i)

        if cameraPoseEnac == None:
            self.get_logger().info("missing reference aruco, can't estimate pose ! ")
            self.__send_diagnostics(DiagnosticStatus.ERROR, "missing reference aruco")
            return 
        for i in aurcosIdsIndex:
            marker_id = aruco_poses.marker_ids[i]
            pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])

            table_pose = calc.table_pos_from_camera(pose, cameraPoseEnac)
            self.get_logger().info(
                f"{marker_id} is located on table at {table_pose}"
            )
            calc.publish_pos_from_reference(self.tf_publisher, now, pose, 'camera', str(marker_id)+"_camera")
            calc.publish_pos_from_reference(self.tf_publisher, now, table_pose, 'aruco', str(marker_id)+"_table")

            """
            if marker_id == 13:
                for x in range(2):
                    x_rad = x*math.pi
                    for y in range(2):
                        y_rad = y*math.pi
                        for z in range(2):
                            z_rad = z*math.pi
                            transf = pose.transform_offset(x_rad, y_rad, z_rad)
                            calc.publish_pos_from_reference(self.tf_publisher, now, transf, 'camera', str(marker_id)+"_camera"+str(x)+str(y)+str(z))
            """

    def __send_diagnostics(self, level, msg_txt):
        if not hasattr(self, 'diagnostics'):
            return #not in debug mode
        msg = DiagnosticArray()
        reference = DiagnosticStatus()
        reference.level = level
        reference.name = "analysis_aruco"
        reference.message = msg_txt
        reference.hardware_id = "camera"
        values = KeyValue()
        values.key = "acceptable aruco reference ids"
        values.value = '<h3>'+ str(self.arucosStorage.reference_ids) + '</h3>'
        reference.values = [values]
        msg.status = [reference]
        self.diagnostics.publish(msg)

    def publish_arucos(self):
        pass


def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
