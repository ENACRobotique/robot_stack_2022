from geometry_msgs.msg import TransformStamped, _pose, _transform
from ros2_aruco_interfaces.msg import _aruco_markers
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import math
import aruco_analysis_enac.aruco_calculations as calc

Pose = _pose.Pose
Transform = _transform.Transform
ArucoMarkers = _aruco_markers.ArucoMarkers


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(x,y,z,q):
    #TODO!!!
    e = [0]*3
    return e

def add_arucos_known(*arucos):
    # [ID_ARUCO, xPos, yPos, zPos, xQuat, yQuat, zQuat, wQuat]
    arucos_transforms = {}
    for aruco in arucos:
        if any(x != 0 for x in aruco):
            print(aruco)
            arucos_transforms[aruco[0]] = Pose()
            arucos_transforms[aruco[0]].position.x = aruco[1]
            arucos_transforms[aruco[0]].position.y = aruco[2]
            arucos_transforms[aruco[0]].position.z = aruco[3]
            arucos_transforms[aruco[0]].orientation.x = aruco[4]
            arucos_transforms[aruco[0]].orientation.y = aruco[5]
            arucos_transforms[aruco[0]].orientation.z = aruco[6]
            arucos_transforms[aruco[0]].orientation.w = aruco[7]

    return arucos_transforms

class ArucoAnalysis(Node):

    def __init__(self):
        super().__init__('aruco_analysis_tf_publisher')

        self.declare_parameter('aruco_topic', '/aruco_markers')

        #Aruco codes must be filled in the order :
        #order of float64:
        #[ID_ARUCO, xPos, yPos, zPos, xQuat, yQuat, zQuat, wQuat]
        #position relative to the table/2d frame
        self.declare_parameter('origin', [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter('precision_increaser_1', [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter('precision_increaser_2', [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter('precision_increaser_3', [0, 0, 0, 0, 0, 0, 0, 0])

        self.aruco_topic = self.get_parameter('aruco_topic').get_parameter_value().string_value
        self.aruco_known = add_arucos_known(
            self.get_parameter('origin').get_parameter_value().double_array_value,
            self.get_parameter('precision_increaser_1').get_parameter_value().double_array_value,
            self.get_parameter('precision_increaser_2').get_parameter_value().double_array_value,
            self.get_parameter('precision_increaser_3').get_parameter_value().double_array_value,
        )

        self.transformPublisher = TransformBroadcaster(self)
        # Initialize the transform broadcaster
        #self.br = TransformBroadcaster(self)
        self.aruco_poses = self.create_subscription(
            ArucoMarkers,
            self.aruco_topic,
            self.__handle_arucos,
            10
        )


    def __PoseROS_to_PoseENAC(self, poseROS:Pose):
        rotation = euler_from_quaternion(poseROS.orientation.x, poseROS.orientation.y, poseROS.orientation.z, poseROS.orientation.w)
        return calc.Pose(poseROS.position.x, poseROS.position.y, poseROS.position.z, rotation[0], rotation[1], rotation[2])

    def __PoseENAC_to_transfStamped(self, timestamp, frame_id:str, poseENAC: calc.Pose):
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

    def __handle_arucos(self, aruco_poses):
        #Analysis is done in two steps : first, we determine camera position, then we analyse the "free" aruco, the one we don't know their position on the table

        now = aruco_poses.header.stamp
        arucoIdsToAnalyse = []
        #toPublish = [] #type : transformStamped[]  - regroup all the transforms to publish at once

        for id, i in enumerate(aruco_poses.marker_ids):
            if id in self.aruco_known:
                cameraPoseENAC = calc.get_camera_position(self.__PoseROS_to_PoseENAC(aruco_poses[i])) #TODO : faire une fusion de données, là on se contente de prendre le dernier
                #toPublish.transforms.append(self.__PoseENAC_to_transfStamped(timestamp=now, poseENAC=cameraPoseENAC))
                self.transformPublisher.sendTransform(
                    self.__PoseENAC_to_transfStamped(timestamp=now, frame_id='camera', poseENAC=cameraPoseENAC))
            else:
                arucoIdsToAnalyse.append(i)

        for i in arucoIdsToAnalyse:
            pass #TODO : publier les transforms des choses à côté




def main():
    rclpy.init()
    node = ArucoAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
