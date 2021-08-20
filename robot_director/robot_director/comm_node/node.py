import robot_director.comm_node.data_types as data_types
import robot_director.comm_node.serial_interface as serial_enac #pour éviter que quelqu'un cherche des infos sur serial alors que la doc existe pas
import robot_director.comm_node.ros_interface as ros_enac
from time import sleep

class Robot():
    """
    store the data used by serial & ros
    Design consideration :
        we consider that the node run fast enough so that the ros_interface use its timestamp when sending data instead of timestamping when serial was received
    """
    def __init__(self):
        self.pos = data_types.PositionOriented(0, 0, 0)

    def set_odom(self, data: data_types.PositionOriented):
        self.pos = data
    def get_odom(self):
        return self.pos

def main(args=None):
    """
    entry points for ros
    """

    #region parameters
    #TODO : transform these into ros parameters (yaml file?)
    robotName = "robot_test"
    paramNbPerSec = 30
    rate = float(1/paramNbPerSec)
    port = '' #TODO : mettre le port !
    baudrate = 115200
    ros_convert_to_mm = True #TODO : implémenter la possibilité de désactiver la conversion de m en mm
    #endregion

    #region initializations
    ser = serial_enac.SerialInterface(port, baudrate=baudrate, timeout=rate) #we use the pyserial timeout (time to spend blocking and reading the serial port) to loop at a certain rate
    ros = ros_enac.RosInterface(robotName)

    #Interraction from ROS side :
    #ros.update_data_continuous('/odom', data_types.PositionOrientedTimed, get_odom, 30)
    #ros.register_msg_callback('/cmd_vel', data_types.Speed, set_speed)
    #ros.register_msg_callback('/cmd_actuator', data_types.Actuator, set_actuator)
    #ros.register_msg_callback('/score', int, set_score)

    #Interraction from embedded microcontroller side:
    #ser.update_data_continuous('o', data_types.PositionOriented, get_odom, 30)
    #ser.register_msg_callback('s', data_types.Speed, set_speed)
    #ser.register_msg_callback('a', data_types.Actuator, set_actuator)

    ser.start()
    ros.start()

    #endregion

    #region loop
    while True:
        ros.process_com()
        ser.process_com()
        #sleep(rate)
    #endregion
