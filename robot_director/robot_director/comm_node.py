import robot_director.comm.data_types as data_types
import robot_director.serial_interface as serial_enac #pour éviter que quelqu'un cherche des infos sur serial alors que la doc existe pas
import robot_director.comm.ros_interface as ros_enac


class Robot():
    """
    forward data between two interfaces, intended between serial and ROS.
    Currently, it also allow to store data (but it's useless as the data is sent ASAP).
    Design consideration :
        we consider that the node run fast enough so that the ros_interface use its timestamp when sending data instead of timestamping when serial was received

    """
    def __init__(self, ser: serial_enac, ros: ros_enac):
        self.pos = data_types.PositionOriented(0, 0, 0) #x, y, theta | mm, mm, rad
        self.speed = (0, 0) #vx, vz | m/s, rad/s
        self.ser = ser
        self.ros = ros

    def send_odom_to_ros(self, data: data_types.PositionOriented):
        self.pos = data
        self.ros.send_data(self.pos)

    def send_speed_to_ser(self, data: data_types.Speed):
        self.speed = data
        self.ser.send_data(self.speed)

def main(args=None):
    """
    entry points for ros
    """
    print("beggiing comm_node initialization parameters ")
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
    robot = Robot(ser, ros)
    #TODO : move interraction to a param file, so it will be easier with 2 robots
    #Interraction - reception from ROS side :
    ros.register_msg_callback('/cmd_vel', data_types.Speed, robot.send_speed_to_ser)
    #ros.register_msg_callback('/cmd_actuator', data_types.Actuator, set_actuator)
    #ros.register_msg_callback('/score', int, set_score)

    #Interraction - reception from embedded microcontroller side:
    #Direction : embedded -> python script
    ser.register_msg_callback('o', data_types.PositionOriented, robot.send_odom_to_ros)
    #ser.register_msg_callback('a', data_types.Actuator, robot.send_actuator_to_ros) #actuator state


    ser.start()
    ros.start()

    #endregion

    #region loop
    while True:
        ros.process_com()
        ser.process_com()
        #sleep(rate)
    #endregion

