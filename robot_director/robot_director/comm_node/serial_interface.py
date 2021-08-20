from robot_director.comm_node.interface import Interface
import robot_director.comm_node.data_types as data_type
import serial

class SerialInterface(Interface):
    def __init__(self, port, timeout = 0.05, baudrate = 115200, rx_buffer_size=64, tx_buffer_size=64): # default buffer size like teensy
        super(Interface, self).__init__()
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.ser.set_buffer_size(rx_buffer_size, tx_buffer_size)

    def serial_to_data_type(self, input:str):
        datas = input.split(" ")
        ser_type = datas[0]
        if ser_type == "o": #example : o 0 0 0     or     o 12.11342 0.424535
            pos = data_type.PositionOriented(float(datas[1]), float(datas[2]), float(datas[3]))
            return pos
        else:
            raise NotImplementedError("type coming from embedded not supported at the moment !")

    def data_type_to_serial(self, data):
        if type(data_type) == data_type.Speed:
            return "s " + str(data.vx) + " " + str(data.vz)
        else:
            raise NotImplementedError("type sent from ros not supported at the moment")
    def start(self, *args):
        #TODO : implémenter une réponse serial, du type nom du robot, capacité,...
        print("serial interface started - didn't check if it was working !")

    def process_com(self):
        reading = self.ser.readline()
        if reading[0]

    def stop(self):
        raise NotImplementedError()

    def update_data_continuous(self, name: str, dataType: data_type, get_data_callback, rate: float):

        raise NotImplementedError()

    def register_msg_callback(self, name: str, dataType: data_type, set_data_callback):

        raise NotImplementedError()