from interfaces_enac.msg import _ObjectMarked
import aruco_analysis_enac.aruco_calculations as arucalc
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

Aruco = _ObjectMarked.ObjectMarked

class ArucosStorage:
    #manage storage & generation of a ObjectMarked message, for arucos
    def __init__(self, frame_id:str, arucos : [Aruco], depth=5, xyMargin=0.005, ):
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
        self.xyMargin = xyMargin

        for x in arucos:
            if x.marker_id in self.arucos:
                self.arucos[x.marker_id][x.object_id] = [x] * depth
            else:
                self.arucos[x.marker_id] = {x.object_id: [x] * depth}

    def add_arucos(self, marker_id : int, positions : [arucalc.Pose], stamp: Time):
        #call it by same marker_id
        self.analyzedArucos = {} #key = object_id
        #add_aruco, and add to analyzedArucos

    def add_aruco(self, marker_id : int, position : arucalc.Pose, stamp: Time):
        aruco = Aruco()
        pose = PoseStamped()

        aruco.marker_id = marker_id

        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id

        q = arucalc.quaternion_from_euler(arucalc.roll, arucalc.pitch, arucalc.yaw)
        pose.pose.position.x = position.x
        pose.pose.position.y = position.y
        pose.pose.position.z = position.z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        aruco.poseStamped = pose

        aruco = self.correlation(aruco) #fill object_id
        arucosHistory = self.arucos[aruco.marker_id][aruco.object_id] #arucos with the same object_id
        aruco.diameter = arucosHistory[0].diameter #diameter has been filled when initiating arucoStorageClass
        arucosHistory.pop(0)
        arucosHistory.append(aruco)

    def distance_squared_two_arucos_ground(self, aruco1: Aruco, aruco2: Aruco):
        return (aruco1.pose.pose.position.x - aruco2.pose.pose.position.x)**2 + (aruco1.pose.pose.position.y - aruco2.pose.pose.position.y)**2

    def correlation(self, aruco : Aruco) -> Aruco:
        #todo : traiter possiblesArucos pour chercher que dedans
        """
        to be used when an aruco code position has been analyzed, when we need to associate it to an object_id

        In aruco, fields that need to be filled :
        Pose(Stamped)
        marker_id

        return aruco, but with object_id filled
       """
        leastDistance = 100000 #abritaty distance really big
        for possibleAruco in self.arucos[aruco.marker_id]:
            dist = self.distance_squared_two_arucos_ground(possibleAruco, aruco)
            if  dist < leastDistance:
                leastDistance = dist
                aruco.object_id = possibleAruco.object_id
        if leastDistance >= self.xyMargin:
            raise Exception(f"aruco marker {aruco.marker_id} is identified as {aruco.object_id} but correlation issue : above distance squared margin ! ")
        return aruco


