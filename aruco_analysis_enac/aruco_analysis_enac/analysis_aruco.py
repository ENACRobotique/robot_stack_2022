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
        now = aruco_poses.header.stamp
        aurcosIdsIndex = [] #not reference
        cameraPoseEnac = None
        for i, id in enumerate(aruco_poses.marker_ids):
            if id in self.arucosStorage.reference_ids:
                cameraPoseEnac = True
                print(aruco_poses.rvecs[i])
                pose = fiducial_to_enac_pose(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                calc.publish_pos_from_reference(self.tf_publisher, now, pose, 'original_camera', 'ref_aruco')
                #print(self.tf_buffer.lookup_transform('original_camera', 'ref_aruco', now))
                #print(self.tf_buffer.lookup_transform('ref_aruco', 'original_camera', rclpy.time.Time()), rclpy.Duration(seconds = 0))
                #cameraPoseEnac = calc.get_camera_position(pose) #TODO : faire une fusion de données, là on se contente de prendre le dernier
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
            pose = fiducial_to_enac_pose(aruco_poses.tvecs[i], aruco_poses.rvecs[i])

            #table_pose = calc.table_pos_from_camera(pose, cameraPoseEnac)
            #self.get_logger().info(
            #    f"{marker_id} is located on table at {table_pose}"
            #)

    def __send_diagnostics(self, level, msg_txt):
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



def fiducial_to_enac_pose(tvec, rvec):
    return calc.Pose(tvec.x, tvec.y, tvec.z, rvec.x, rvec.y, rvec.z)

def enac_pose_to_ros_pose(self, timestamp, frame_id:str, poseENAC: calc.Pose):
    transf = TransformStamped()
    transf.header = Header()
    transf.header.stamp = timestamp
    transf.header.frame_id = 'map'
    transf.child_frame_id = frame_id

    q = quaternion_from_euler(poseENAC.roll, poseENAC.pitch, poseENAC.yaw)
    transf.transform = Transform()
    transf.transform.translation.x = poseENAC.x
    transf.transform.translation.y = poseENAC.y
    transf.transform.translation.z = poseENAC.z
    transf.transform.rotation.x = q[0]
    transf.transform.rotation.y = q[1]
    transf.transform.rotation.z = q[2]
    transf.transform.rotation.w = q[3]
    return transf

def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
