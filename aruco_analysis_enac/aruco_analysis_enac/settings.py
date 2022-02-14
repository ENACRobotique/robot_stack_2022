from math import pi

#TODO : class Arucos, with function regroup_corners_by_size, & property img_analysis_frequency

class Aruco():
    def __init__(self, id, size=0.07, position = [0,0,0], rotation=[0,0,0]) -> None:
        self.id = id
        self.position = position
        self.rotation = rotation

def generate_aruco_subset(id_begin, id_end, size=0.07) -> list(Aruco):
    arucos = []
    for i in range(id_begin, id_end):
        arucos.append(Aruco(i, size))


#subset 1
robot_subset_1 = [
    Aruco(42, 0.10, [1.50, 1.0, 0.0], [0.0, 0.0, pi]), #180° rotation of 42 compared to camera (or origin)

]
robot_subset_1.append(generate_aruco_subset(3,6)) #equipe bleue ou qq chose comme ça


#Function to call to get arucos markers subsets using ros_args in detect_aruco for instance
#Usage : 
    #from aruco_analysis_enac.settings import get_markers
    #ARUCOS_TO_DETECT = get_markers(self.get_parameter('aruco_subset').get_parameter_value().int_value)
def get_markers(subset=1):
    if subset == 1:
        return robot_subset_1
    else:
        Exception("unsupported subset requested with ros args")