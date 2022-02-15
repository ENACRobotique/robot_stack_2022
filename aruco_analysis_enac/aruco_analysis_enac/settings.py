from functools import lru_cache
from math import pi
from enum import IntFlag
        
class Movement(IntFlag):
    MOVING=1,
    ROCK=2,
    FIXED=4,
    ALL=7

class Aruco():
    def __init__(self, id, size=0.07, position = [0,0,0], rotation=[0,0,0], expected_mvt = Movement.MOVING) -> None:
        """[summary]

        Args:
            id ([type]): [description]
            size (float, optional): [description]. Defaults to 0.07.
            position (list, optional): [description]. Defaults to [0,0,0]
            rotation (list, optional): [description]. Defaults to [0,0,0].
            frameInterval (int, optional): For optimisation : time between calls. Defaults to 0.
            isFixed (bool, optional): [description]. Defaults to False.
        """
        self.id = id
        self.position = position
        self.rotation = rotation
        self.size = size
        self.expected_mvt = expected_mvt

class ArucosSetting():
    def __init__(self, arucos: list) -> None:
        self.arucos = arucos
        pass

    @lru_cache(maxsize=1000)
    def get_aruco_by_id(self, id:int) -> Aruco:
        """
        Get the aruco with the id given in argument
        """
        for aruco in self.arucos:
            if aruco.id == id:
                return aruco
        return None

    def regroup_corners_by_size(self, corners, ids, logger=None, mvt = Movement.ALL) -> dict:
        """
        Regroup the corners by size for each aruco in order to estimate the pose of the aruco.

        Args:
            corners ([type]): [description]
            ids ([type]): [description]
            logger ([type], optional): [description]. Defaults to None.
            frame_stamp (int, optional): [description]. Defaults to -1 -> return all corners.

        Returns:
            dict: [description] example : {size: [[corners], [ids]], ... }
        """
        
        if len(corners) != len(ids):
            logger.error("Error : corners and ids are not the same size")
            return None
        #extract the corners of each aruco depending on the size, each corner array is made of 4 points
        corners_by_size = {}
        for i, id in enumerate(ids):
            aruco = self.get_aruco_by_id(id[0])
            if aruco.expected_mvt in mvt:
                corners_by_size[aruco.size] = [[corners[i]],[id[0]]]

        return corners_by_size


def generate_aruco_subset(id_begin, id_end, size=0.07, expected_mvt = Movement.MOVING) -> list:
    arucos = []
    for i in range(id_begin, id_end):
        arucos.append(Aruco(i, size, expected_mvt=expected_mvt))
    return arucos

#subset 1
robot_subset_1 = [
    Aruco(42, 0.10, [1.50, 1.0, 0.0], [0.0, 0.0, pi]), #180° rotation of 42 compared to camera (or origin)

]
robot_subset_1.append(generate_aruco_subset(3,6)) #equipe bleue ou qq chose comme ça
aruco1 = ArucosSetting(robot_subset_1)


#Function to call to get arucos markers subsets using ros_args in detect_aruco for instance
#Usage : 
    #from aruco_analysis_enac.settings import get_markers 
    #ARUCOS_TO_DETECT = get_markers(self.get_parameter('aruco_subset').get_parameter_value().int_value)
def get_markers(subset=1):
    if subset == 1:
        return aruco1
    else:
        Exception("unsupported subset requested with ros args")

marker_analysis_frame_interval = {
    Movement.FIXED: 10,
    Movement.ROCK: 10,
    Movement.MOVING:1,
    }
def get_movement_flag(frame_count:int)->Movement:
    """
    Get the movement flag depending on the frame count.
    """
    flag = 0
    for mvt in marker_analysis_frame_interval:
        if frame_count % marker_analysis_frame_interval[mvt] == 0:
            flag |= mvt
    return flag