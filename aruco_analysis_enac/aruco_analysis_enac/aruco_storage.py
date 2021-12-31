from interfaces_enac.msg import _object_marked
import aruco_analysis_enac.aruco_calculations as arucalc
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from rclpy import logging

Aruco = _object_marked.ObjectMarked


def create_aruco_msg_uncorel(marker_id : int, position : arucalc.Pose, stamp: Time, frame_id="map"):
    """
    diameter, is_moving, object_id unfilled !
    """
    aruco = Aruco()
    pose = PoseStamped()

    aruco.marker_id = marker_id

    pose.header.stamp = stamp
    pose.header.frame_id = frame_id

    q = arucalc.quaternion_from_euler(position.roll, position.pitch, position.yaw)
    pose.pose.position.x = float(position.x)
    pose.pose.position.y = float(position.y)
    pose.pose.position.z = float(position.z)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    aruco.pose = pose

    return aruco

class ArucosStorage:
    #manage storage & generation of a ObjectMarked message, for arucos
    def __init__(self,  arucos : [Aruco], frame_id="map", depth=5, xyMargin=0.005, logger="comm_node"):
        #depth : number of positions kept for correlation
        #xyMarking : for correlation, in meters, distance allowed to move

        self.frame_id = frame_id
        self.arucos = {}
        """
        {marker_id:
            {
            object_id: [Aruco]*depth,
            object_id: [Aruco]*depth, ...
            },
        marker_id: ...
        """
        self.depth = depth
        self.xyMargin = xyMargin

        for x in arucos:
            if x.marker_id in self.arucos:
                self.arucos[x.marker_id][x.object_id] = [x] * depth
            else:
                self.arucos[x.marker_id] = {x.object_id: [x] * depth}

        self.logger = logging.get_logger(logger)

    def add_arucos(self, marker_id : int, positions : [arucalc.Pose], stamp: Time):
        #call it by same marker_id
        analyzedArucos = {} #key = object_id
        #add_aruco, and add to analyzedArucos

    def add_aruco(self, aruco:Aruco): #create using method create_aruco_msg_uncorel
        #aruco = create_aruco_msg_uncorel(marker_id, position, stamp, self.frame_id)
        aruco = self.correlation(aruco) #fill object_id
        arucosHistory = self.arucos[aruco.marker_id][aruco.object_id] #arucos with the same object_id
        aruco.diameter = float(arucosHistory[0].diameter) #diameter has been filled when initiating arucoStorageClass
        arucosHistory.pop(0)
        arucosHistory.append(aruco)

    def has_aruco_moved(self, marker_id: int, object_id:str):
        """
        #warning : support moving only for 2 last "pictures"
        """
        history = self.arucos[marker_id][object_id]
        if self.distance_squared_two_arucos_ground(history[-1], history[-2]) >= self.xyMargin:
            return True
        return False

    def distance_squared_two_arucos_ground(self, aruco1: Aruco, aruco2: Aruco):
        return (aruco1.pose.pose.position.x - aruco2.pose.pose.position.x)**2 + (aruco1.pose.pose.position.y - aruco2.pose.pose.position.y)**2

    def correlation(self, aruco : Aruco) -> Aruco:
        """
        to be used when an aruco code position has been analyzed, when we need to associate it to an object_id

        In aruco, fields that need to be filled :
        Pose(Stamped)
        marker_id

        return aruco, but with object_id & is_moving filled
       """

        leastDistance = 100000 #abritaty distance really big
        if self.arucos.get(aruco.marker_id, None) == None:
            self.logger.info(f"Tried to correlate newly detected marker with ID : | {aruco.marker_id} | - Starting to track it")
            aruco.object_id = "unknown"
            #TODO : implement a function to look for a potential object_id in settings.py
            self.arucos[aruco.marker_id] = {aruco.object_id: [aruco] * self.depth}
            aruco.is_moving = True
            return aruco


        for possibleAruco in self.arucos[aruco.marker_id].values():
            dist = self.distance_squared_two_arucos_ground(possibleAruco[-1], aruco)
            if  dist < leastDistance:
                leastDistance = dist
                aruco.object_id = possibleAruco[-1].object_id
        if leastDistance >= self.xyMargin:
            self.logger.info(f"aruco marker {aruco.marker_id} is identified as {aruco.object_id} and moving above xymargin ")
            aruco.is_moving = True
        else:
            self.logger.info(f"aruco marker {aruco.marker_id} is identified as {aruco.object_id} and not moving ")
            aruco.is_moving = False

        return aruco


