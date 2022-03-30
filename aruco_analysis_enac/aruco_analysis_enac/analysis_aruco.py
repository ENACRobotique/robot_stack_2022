from turtle import position
from aruco_analysis_enac.tf import Transform
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


import aruco_analysis_enac.aruco_calculations as calc
from aruco_analysis_enac.aruco_storage import ArucosStorage
from aruco_analysis_enac.settings import Movement as Mvt
from interfaces_enac.msg import _fiducials_poses, _object_marked#, _objects_marked

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster


FidPoses = _fiducials_poses.FiducialsPoses
ObjectMarked = _object_marked.ObjectMarked
#ObjectsMarked = _objects_marked.ObjectsMarked

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
        ref_aruco_transforms = {} #id: pose wrt origin

        #first step : get possible camera positions from all the reference arucos
        for i, id in enumerate(aruco_poses.marker_ids):
            if id in self.arucosStorage.reference_ids:
                pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])

            
                #TODO : take from aruco storage add middle
                ref_aruco = self.arucosStorage.get_ref_aruco(id)
                ref_aruco_transform = calc.Transform.from_position_euler(*ref_aruco.position, *ref_aruco.rotation) #unpack position with x,y,z and rotation with  (euler) x,y,z
                cur_cam_pose = calc.get_camera_position(pose)
                cam_poses[id] = cur_cam_pose
                ref_aruco_transforms[id] = ref_aruco_transform
                calc.publish_pos_from_reference(self.tf_publisher, now, ref_aruco_transform, "origin", calc.str_ref_aruco(id))

                calc.publish_pos_from_reference(self.tf_publisher, now, cur_cam_pose,
                    calc.str_ref_aruco(id), calc.str_camera(id)
                )
                self.get_logger().info(
                    f"according to reference {id}, camera is at {cur_cam_pose.position}"
                )
            else:
                aurcosIdsIndex.append(i)

        if cam_poses == []:
            self.get_logger().info("missing reference aruco, can't estimate pose ! ")
            self.__send_diagnostics(DiagnosticStatus.ERROR, "missing reference aruco")
            return 
        objects_marked = []

        #second step : iterate over all the aruco not reference to estimate their position
        for cam_ref_id, camera_pose in cam_poses.items():
            for i in aurcosIdsIndex:
                marker_id = aruco_poses.marker_ids[i]
                pose = calc.Pose.from_tvec_rvec(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                rel_marker_pose = calc.pos_wrt_marker_from_camera(pose, camera_pose)
                marker_pose_wrt_origin = calc.pose_wrt_origin(ref_aruco_transforms[cam_ref_id], rel_marker_pose)
                self.get_logger().info(
                    f"{marker_id} is located on table at {marker_pose_wrt_origin.position} according to reference {cam_ref_id}"
                )

                calc.publish_pos_from_reference(self.tf_publisher, now, pose.transform_offset(), 
                    calc.str_camera(cam_ref_id),
                    calc.str_camera_aruco(cam_ref_id, marker_id)
                )
                calc.publish_pos_from_reference(self.tf_publisher, now, rel_marker_pose, 
                calc.str_ref_aruco(cam_ref_id), calc.str_ref_aruco_to_aruco(cam_ref_id, marker_id) 
                )

                #third step : publish the object poses
                #TODO : avoid dupplication of marker ids if multiple references
                objects_marked.append(self.__get_object_marked_msg(aruco_poses.header, marker_id, marker_pose_wrt_origin))
        #self.__publish_objects_marked(objects_marked) #TODO : uncomment

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

    #def identify_fixed_markers(self, )
    def publish_arucos(self, objects_marked):
        """Publish aruco poses in ObjectMarked """
        #cur_obj = ObjectsMarked()
        #cur_obj.objectsMarked = objects_marked
        #self.object_marked_pub.publish(cur_obj)
        pass

    def __get_object_marked_msg(self, header, id:int, pose:Transform):
        """ get a ObjectMarked message from a aruco id and a pose"""
        cur_obj = ObjectMarked()
        cur_obj.marker_id = id
        cur_obj.is_moving = True if self.arucosStorage.get_aruco_mvt(id) == Mvt.MOVING else False
        
        #fill PoseStamped message
        cur_obj.pose.header = header
        #cur_obj.pose.pose.position = pose.position #TODO uncomment
        #cur_obj.pose.pose.orientation = pose.quaternion
        #TODO : fill diameter with settings.py
        return cur_obj
def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()