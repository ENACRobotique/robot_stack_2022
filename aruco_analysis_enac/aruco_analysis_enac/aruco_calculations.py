import math

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


class Pose:
    def __init__(self, x,y,z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self):
        return f"Pos : {self.x:.2f} {self.y:.2f} {self.z:.2f} \n \
            Rotation : {self.roll:.2f} {self.pitch:.2f} {self.yaw:.2f} \n"

def get_camera_position(arucoRef : Pose):
    x = -arucoRef.x
    y = -arucoRef.y
    z = arucoRef.z
    roll = arucoRef.roll
    pitch = arucoRef.pitch
    yaw = arucoRef.yaw

    return arucoRef
    #return (x, y, z, roll, pitch, yaw)
    #position de la caméra par rapport au marqueur qui sert d'origine

def table_pos_from_camera(arucoPosition: Pose, cameraPosition: Pose):
    """
    return aruco position on table from camera position on table, using only one aruco code as reference
    arucoPosition : relative to camera
    camera Position : relative to table
    """
    return arucoPosition
    

