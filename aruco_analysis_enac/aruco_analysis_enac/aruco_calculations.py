class Pose:
    def __init__(self, x,y,z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

def get_camera_position(arucoRef : Pose):
    x = arucoRef.x
    y = arucoRef.y
    z = arucoRef.z
    roll = arucoRef.roll
    pitch = arucoRef.pitch
    yaw = arucoRef.yaw

    return arucoRef
    #return (x, y, z, roll, pitch, yaw)
    #position de la caméra par rapport au marqueur qui sert d'origine
