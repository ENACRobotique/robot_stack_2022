from aruco_analysis_enac.aruco_storage import Aruco, create_aruco_msg_uncorel
from aruco_analysis_enac.aruco_calculations import Pose as enacPose
from builtin_interfaces.msg import Time

xyMargin = 0.002 #aruco is considered moving between two frames above this value in meters

def generate_aruco_correlation(aruco:Aruco, object_id:str, is_moving=True, diameter=0.0): #diameter 0 : not an obstacle on ground level
    aruco.object_id = object_id
    aruco.is_moving = is_moving
    aruco.diameter = float(diameter)
    return aruco

arucoInMiddlePose = enacPose(1.5, 1.0, 0, 0, 0, 0)
arucoInMiddle = create_aruco_msg_uncorel(42, arucoInMiddlePose, Time(sec=1), "map")
arucoInMiddle = generate_aruco_correlation(arucoInMiddle, "origin", False, 0)

arucos = [
    arucoInMiddle,
]