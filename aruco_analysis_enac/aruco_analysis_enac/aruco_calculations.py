from functools import cached_property, lru_cache
import math
#from aruco_analysis_enac.aruco_storage import aruco_storage
import numpy as np
from aruco_analysis_enac.tf import TFTree, TFNode, Transform
import aruco_analysis_enac.transformations as tft
from cv2 import Rodrigues, decomposeProjectionMatrix

from geometry_msgs.msg import TransformStamped

def rotationMatrixToEulerAngles(R):

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])
class Aruco:
    def __init__(self, id:int, x:float, y:float, z:float) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.z = z
class Pose:
    """Autoflip Z axis in case of ambiguity problem for arucos :
    https://github.com/opencv/opencv/issues/8813#issuecomment-390462446
    """
    def __init__(self, x,y,z, rvec):
        self.x = x
        self.y = y
        self.z = z
        self.rvec = rvec

    @classmethod
    def from_tvec_rvec(cls, tvec, rvec):
        return cls(tvec.x, tvec.y, tvec.z, np.float32([rvec.x, rvec.y, rvec.z]))

    @cached_property
    def transform(self):
        T = (self.x, self.y, self.z)
        R = tft.identity_matrix()
        rot_3 = Rodrigues(self.rvec)[0]
        #Why not roll pitch yaw : https://github.com/microsoft/AirSim/issues/1806
        (yaw, pitch, roll) = rotationMatrixToEulerAngles(rot_3)
        #roll pitch yaw = (0,1,2) --> (2,1,0)
        t = Transform.from_position_euler(self.x, self.y, self.z, roll+math.pi, pitch, yaw)
        
        """
        rot_matrix = np.eye(4)
        rot_matrix[0:3, 0:3] = Rodrigues(self.rvec)[0]
        quat = tft.quaternion_from_matrix(rot_matrix)
        print(rot_matrix)
        t = Transform(self.x, self.y, self.z, quat[0],quat[1], quat[2], quat[3])
        print(t.euler)
        """
        return t

    def transform_offset(self, xAxis, yAxis, zAxis):
        rot_3 = Rodrigues(self.rvec)[0]
        #Why not roll pitch yaw : https://github.com/microsoft/AirSim/issues/1806
        #roll pitch yaw = (0,1,2) --> (2,1,0)
        (yaw, pitch, roll) = rotationMatrixToEulerAngles(rot_3)
        t = Transform.from_position_euler(self.x, self.y, self.z, roll+xAxis, pitch+yAxis, yaw+ zAxis)
        return t
    
    def __str__(self):
        return f"Pos : {self.x:.2f} {self.y:.2f} {self.z:.2f} \
            Rotation : {self.roll} {self.pitch} {self.yaw} \n"

def quaternion_from_euler(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def publish_pos_from_reference(publisher, time, arucoPose, parent, child):
    """[summary]

    Args:
        publisher ([type]): [description]
        time ([type]): [description]
        arucoPose ([Pose, Transform]): [description]
        parent ([type]): [description]
        child ([type]): [description]
    """
    trans = TransformStamped()
    trans.header.stamp = time
    trans.header.frame_id = parent
    trans.child_frame_id = child
    trans.transform.translation.x = float(arucoPose.x)
    trans.transform.translation.y = float(arucoPose.y)
    trans.transform.translation.z = float(arucoPose.z)
    if type(arucoPose) == Pose:
        qw, qx, qy, qz = arucoPose.transform.quaternion
    elif type(arucoPose) == Transform:
        qw, qx, qy, qz = arucoPose.quaternion
    trans.transform.rotation.w = qw
    trans.transform.rotation.x = qx
    trans.transform.rotation.y = qy
    trans.transform.rotation.z = qz
    publisher.sendTransform(trans)

def transform_flip_x(transf: Transform):
    euler = list(transf.euler[:])
    #euler[0] += 1.58
    #euler[2] += 0.9
    #euler[0] += 0.9
    euler[2] += math.pi/2
    return Transform.from_position_euler(transf.x, transf.y, transf.z, 
        euler[0], euler[1], euler[2]) 

@lru_cache(maxsize=10)
def get_camera_position(origin_to_aruco: Pose, aruco_to_camera : Pose)-> Transform:
    """ 
    return camera position related to origin of the map (bottom left for eurobot)
    """
    #aruco_to_camera_flipped = transform_flip_x(aruco_to_camera.transform)
    tree = TFTree()
    #tree.add_transform("origin", "aruco", origin_to_aruco.transform)
    tree.add_transform("aruco", "camera", aruco_to_camera.transform)
    print("camera as a parent")
    print(tree.lookup_transform("camera", "aruco").euler)
    print("aruco as a parent")
    print(tree.lookup_transform("aruco", "camera").euler)
    return aruco_to_camera.transform #tree.lookup_transform("camera", "aruco")
    #ref_transform = Transform.from_position_euler(arucoRef.x,arucoRef.y, arucoRef.z, arucoRef.roll, arucoRef.pitch, arucoRef.yaw)
    #print(tft.translation_from_matrix(tft.inverse_matrix(ref_transform.matrix)))
    #invTvec = np.dot(R, np.matrix(-tvec))
    #invRvec, _ = Rodrigues(R)
    #rot_mat = np.matrix(rot_mat).T
    #inverted = np.linalg.inv(transform_matrix)
    

def table_pos_from_camera(arucoPosition: Pose, cameraPosition: Transform)-> Transform:
    """
    return aruco position on table from camera position on table, using only one aruco code as reference
    arucoPosition : relative to camera
    camera Position : relative to table
    """
    #transf_aruco = transform_flip_x(arucoPosition.transform)
    tree = TFTree()
    tree.add_transform("ref","cam", cameraPosition)
    tree.add_transform("cam","moving", arucoPosition.transform_offset(0,0,0))
    transf = tree.lookup_transform("ref","moving")
    return transf


def rotation(theta1,theta2,theta3,v) :
    Mx = np.array([[1,0,0],[0,math.cos(theta1),-math.sin(theta1)],[0,math.sin(theta1),math.cos(theta1)]])
    My = np.array([[math.cos(theta2),0,-math.sin(theta2)],[0,1,0],[math.sin(theta2),0,math.cos(theta2)]])
    Mz = np.array([[math.cos(theta3),-math.sin(theta3),0],[math.sin(theta3), math.cos(theta3),0],[0,0,1]])
    res = np.dot(np.dot(np.dot(Mx,My),Mz),v)
    return res

if __name__ == "__main__":
    print(rotation(math.pi,0,0,[0,1,0]))
"""
tvecs:
- x: -0.2657684973603784
  y: 0.28864516818472313
  z: 0.7879794911990743
rvecs:
- x: 2.0277491842572615
  y: 2.0202839337174843
  z: 0.6653893427159006

  get_camera_position(Pose(-0.26,0.28,0.78, 2.02, 2.02, 0.665))

  """