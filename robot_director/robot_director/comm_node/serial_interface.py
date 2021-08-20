from robot_director.comm_node.interface import Interface
import robot_director.comm_node.data_types as data_type
import serial
from typing import Type

#commands with direction python script -> embedded
#for supporting additionnals commands, check data_type_to_serial()
CMD_VEL = "s {} {}" #x speed, theta speed


class SerialInterface(Interface):
    def __init__(self, port, timeout = 0.05, baudrate = 115200, rx_buffer_size=64, tx_buffer_size=64): # default buffer size like teensy
        super(Interface, self).__init__()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)
        self.cbs = {} #dict of callbacks : {data_type: function_to_call} | example : {positionOriented : robot.send_odom_to_ros}
        self.letter_to_type = {} #dict of {char : data_type} | example : {'o' : PositionOriented}

    def get_serial_msg_type(self, input:str):
        data = input.split(" ")
        return self.letter_to_type[input[0]]

    def serial_to_data_type(self, input:str):
        data = input.split(" ")
        dataType = self.get_serial_msg_type(input)
        if dataType == data_type.PositionOriented:
            return data_type.PositionOriented(float(data[1]), float(data[2]), float(data[3]))
        else:
            raise NotImplementedError('data from serial unsupported : first letter problem !')



    def start(self, *args):
        #TODO : implémenter une réponse serial, du type nom du robot, capacité,...
        print("serial interface started - didn't check if it was working !")

    def process_com(self):
        reading = str(self.ser.readline())
        #call the function linked to the serial msg received, and pass it the parameters :
        self.cbs[self.get_serial_msg_type(reading)](self.serial_to_data_type(reading))

    def stop(self):
        raise NotImplementedError()

    def data_type_to_serial(self, data):
        if type(data_type) == data_type.Speed:
            return CMD_VEL.format(data.vx, data.vz)
        else:
            raise NotImplementedError("type sent from ros not supported at the moment")

    def send_data(self, data: data_type):
        self.ser.write(self.data_type_to_serial(data))

    def register_msg_callback(self, name: str, dataType: Type[data_type], set_data_callback):
        self.cbs[dataType] = set_data_callback