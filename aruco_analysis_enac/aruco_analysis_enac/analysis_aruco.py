import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from interfaces_enac.msg import _aruco_markers

import aruco_analysis_enac.aruco_calculations as calc
from aruco_storage import ArucosStorage

ArucoMarkers = _aruco_markers.ArucoMarkers

class ArucoAnalysis(Node):

    def __init__(self, arucoStorage = None, name='aruco_analysis', fixedArucoRate=1.0, movingArucoRate=0.1):
        super().__init__(name=name)

        if arucoStorage:
            self.arucosStorage = arucoStorage
        else:
            self.arucosStorage = ArucosStorage(
                [calc.Aruco(42, 0.15, 0.10, 0.0)], 
                'map')

        self.aruco_poses = self.create_subscription(
            ArucoMarkers,
            '/aruco_poses',
            self.__handle_arucos,
            10
        )

        #TODO : faire une classe à part pour la gestion des publishers avec 2 rate différents
        self.object_poses_publisher = self.create_publisher(Aruco, 'object_poses', 10)
        self.create_timer(movingArucoRate, self.object_poses_publisher)

    
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
                pose = fiducial_to_enac_pose(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
                cameraPoseEnac = calc.get_camera_position(pose) #TODO : faire une fusion de données, là on se contente de prendre le dernier
                self.get_logger().info(
                    f"according to reference {id}, camera is at {cameraPoseEnac}"
                )
            else:
                aurcosIdsIndex.append(i)

        if cameraPoseEnac == None:
            self.get_logger().info("missing reference aruco, can't estimate pose ! ")
            return 
        for i in aurcosIdsIndex:
            marker_id = aruco_poses.marker_ids[i]
            pose = fiducial_to_enac_pose(aruco_poses.tvecs[i], aruco_poses.rvecs[i])
            table_pose = calc.table_pos_from_camera(pose, cameraPoseEnac)
            self.get_logger().info(
                f"{marker_id} is located on table at {table_pose}"
            )

    def publish_arucos(self):
        pass



def fiducial_to_enac_pose(tvec, rvec):
    return calc.Pose(tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2])

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
