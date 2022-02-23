from turtle import position
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


import aruco_analysis_enac.aruco_calculations as calc
from aruco_analysis_enac.aruco_storage import ArucosStorage
from interfaces_enac.msg import _fiducials_poses, _object_marked

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


FidPoses = _fiducials_poses.FiducialsPoses
ObjectMarked = _object_marked.ObjectMarked

class ArucoAnalysis(Node):
    """ class that"""
    def __init__(self, arucoStorage = None, name='aruco_analysis', fixedArucoRate=1.0, movingArucoRate=0.1):
        super().__init__(name)

        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.declare_parameter('subset', 1)
        self.subset = self.get_parameter('subset').get_parameter_value().integer_value

        self.arucosStorage = ArucosStorage.from_subset(self.subset)

        self.aruco_poses = self.create_subscription(
            FidPoses,
            '/aruco_poses',
            self.__handle_arucos,
            10
        )

        self.tf_publisher = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        #TODO : faire une classe à part pour la gestion des publishers avec 2 rate différents
        #self.object_poses_publisher = self.create_publisher(Aruco, 'object_poses', 10)
        #self.create_timer(movingArucoRate, self.object_poses_publisher)

        self.object_marked_pub = self.create_publisher(ObjectMarked, 'object_marked', qos_profile_sensor_data)
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

        now = aruco_poses.header.stamp
        aurcosIdsIndex = [] #not reference
        cam_poses = {} #store multiple camera pose estimated from all the reference aruco
        for i, id in enumerate(aruco_poses.marker_ids):
            if id in self.arucosStorage.reference_ids:
                pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                
                cur_cam_pose = calc.get_camera_position( pose)
                cam_poses[id] = cur_cam_pose
                #TODO : take from aruco storage add middle
                position_wrt_origin = self.arucosStorage.get_ref_aruco(id)
                origin = calc.Transform.from_position_euler(*position_wrt_origin.position, *position_wrt_origin.rotation) #unpack position with x,y,z and rotation with  (euler) x,y,z
                calc.publish_pos_from_reference(self.tf_publisher, now, origin, "origin", calc.str_ref_aruco(id))

                calc.publish_pos_from_reference(self.tf_publisher, now, cur_cam_pose,
                    calc.str_ref_aruco(id), calc.str_camera(id)
                )
                self.get_logger().info(f"camera see {id} at {pose}")
                self.get_logger().info(
                    f"according to reference {id}, camera is at {cur_cam_pose.position}"
                )
            else:
                aurcosIdsIndex.append(i)

        if cam_poses == []:
            self.get_logger().info("missing reference aruco, can't estimate pose ! ")
            self.__send_diagnostics(DiagnosticStatus.ERROR, "missing reference aruco")
            return 
        for cam_ref_id, camera_pose in cam_poses.items():
            for i in aurcosIdsIndex:
                marker_id = aruco_poses.marker_ids[i]
                pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                table_pose = calc.table_pos_from_camera(pose, camera_pose)
                self.get_logger().info(
                    f"{marker_id} is located on table at {table_pose} according to reference at {camera_pose.position}"
                )
                calc.publish_pos_from_reference(self.tf_publisher, now, pose.transform_offset(), 
                    calc.str_camera(cam_ref_id),
                    calc.str_camera_aruco(cam_ref_id, marker_id)
                )
                calc.publish_pos_from_reference(self.tf_publisher, now, table_pose, 
                calc.str_ref_aruco(cam_ref_id), calc.str_ref_aruco_to_aruco(cam_ref_id, marker_id) 
                )

                #calc.publish_pos_from_reference(self.tf_publisher, now, table_pose, 'aruco', str(marker_id)+"_table")

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

    def publish_arucos(self, stamp, id, pose):
        """Publish aruco poses in ObjectMarked """
        cur_obj = ObjectMarked()
        #cur_obj.header.stamp = rospy.Time.now()
        self.object_marked_pub.publi
        pass


def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()