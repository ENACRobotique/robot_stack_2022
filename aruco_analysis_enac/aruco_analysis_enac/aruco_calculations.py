import math
import numpy as np
from aruco_analysis_enac.tf import TFTree, TFNode, Transform

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

def get_camera_position(arucoRef : Pose):
    tree = TFTree()
    ref_transform = Transform.from_position_euler(arucoRef.x,arucoRef.y, arucoRef.z, arucoRef.roll, arucoRef.pitch, arucoRef.yaw)
    tree.add_transform("ref", "cam", ref_transform)
    transf = tree.lookup_transform("ref","cam")
    print("target : ")
    print(ref_transform.position)
    print(ref_transform.euler)
    print(transf.position)
    print(transf.euler)

    return transf
    """
    #position de la caméra par rapport au marqueur qui sert d'origine
    x = -arucoRef.x
    y = -arucoRef.y
    z = -arucoRef.z
    roll = rotation(-arucoRef.roll, 0, 0, [x, y, z]) #x
    pitch = rotation(0, -arucoRef.pitch, 0, roll) #y
    yaw = rotation(0, 0, -arucoRef.yaw, pitch) #z
    #print("calculated rvec :")
    #print(roll)
    #print(pitch)
    #print(yaw)
    return Pose(x,y,z,roll,pitch,yaw)
    """
    

def table_pos_from_camera(arucoPosition: Pose, cameraPosition: Pose):
    tree = TFTree()
    camera_transform = Transform.from_position_euler(cameraPosition.x,cameraPosition.y, cameraPosition.z, cameraPosition.roll, cameraPosition.pitch, cameraPosition.yaw)
    aruco_transform = Transform.from_position_euler(arucoPosition.x,arucoPosition.y, arucoPosition.z, arucoPosition.roll, arucoPosition.pitch, arucoPosition.yaw)
    tree.add_transform("ref", "cam", camera_transform)
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