import math
import numpy as np
from aruco_analysis_enac.tf import TFTree, TFNode, Transform
import aruco_analysis_enac.transformations as tft
from cv2 import Rodrigues

from geometry_msgs.msg import TransformStamped


class Aruco:
    def __init__(self, id:int, x:float, y:float, z:float) -> None:
        self.id = id
        self.x = x
        self.y = y
        self.z = z
class Pose:
    def __init__(self, x,y,z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    
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
    trans = TransformStamped()
    trans.header.stamp = time
    trans.header.frame_id = parent
    trans.child_frame_id = child
    trans.transform.translation.x = float(arucoPose.x)
    trans.transform.translation.y = float(arucoPose.y)
    trans.transform.translation.z = float(arucoPose.z)
    qw, qx, qy, qz = quaternion_from_euler(arucoPose.roll, arucoPose.pitch, arucoPose.yaw)
    trans.transform.rotation.w = qw
    trans.transform.rotation.x = qx
    trans.transform.rotation.y = qy
    trans.transform.rotation.z = qz
    publisher.sendTransform(trans)

def get_camera_position(arucoRef : Pose):

    tvec = np.float32([arucoRef.x, arucoRef.y, arucoRef.z])
    rvec = np.float32([arucoRef.roll, arucoRef.pitch, arucoRef.yaw])
    tvec.reshape((3,1))
    rvec.reshape((3,1))

    R, _ = Rodrigues(rvec)
    R = np.matrix(R).T
    print(R)
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = Rodrigues(R)
    print("target :")
    print(invTvec)
    print(invRvec)
    rot_mat, _ = Rodrigues()
    rot_mat = np.matrix(rot_mat).T
    
    """
    transform_matrix = np.zeros((4,4), dtype=float)
    transform_matrix[0:3, 0:3] = rot_mat
    transform_matrix[0:3, 3] = tvec
    transform_matrix[3,3] = 1
    inverted = np.linalg.inv(transform_matrix)
    invert_rot_mat = inverted[0:3, 0:3]

    tvec = inverted[0:3,3]
    rvec, _ = Rodrigues(invert_rot_mat)
    """
    

    ref_transform = Transform.from_position_euler(arucoRef.x,arucoRef.y, arucoRef.z, arucoRef.roll, arucoRef.pitch, arucoRef.yaw)
    print(tft.translation_from_matrix(tft.inverse_matrix(ref_transform.matrix)))
    print("ref")
    print(ref_transform.position)
    print(ref_transform.euler)
    return ref_transform
    

def table_pos_from_camera(arucoPosition: Pose, cameraPosition: Transform):
    tree = TFTree()
    aruco_transform = Transform.from_position_euler(arucoPosition.x,arucoPosition.y, arucoPosition.z, arucoPosition.roll, arucoPosition.pitch, arucoPosition.yaw)
    tree.add_transform("ref", "cam", cameraPosition)
    tree.add_transform("cam","moving", aruco_transform)
    transf = tree.lookup_transform("ref","moving")
    print("moving : ")
    print(aruco_transform.position)
    print(aruco_transform.euler)
    print(transf.position)
    print(transf.euler)

    return transf
    """
    return aruco position on table from camera position on table, using only one aruco code as reference
    arucoPosition : relative to camera
    camera Position : relative to table
    """
    return arucoPosition


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