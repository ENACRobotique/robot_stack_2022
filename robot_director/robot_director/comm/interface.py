

#from actuators import Actuators, RWType
from robot_director.comm.data_types import PositionOriented, StrMsg, data_type




class Interface():
    """
        Abstract class that needs to be inherited with any "outside" connection/interface (Serial, Ivy,..)
        in order to communicate with the simulator
    """

    def __init__(self):
        pass

    
    def start(self, *args):
        raise NotImplementedError()

    def process_com(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def send_data(self, data: data_type):
        raise NotImplementedError()
    """
    def update_data_continuous(self, name : str, dataType: data_type, get_data_callback, rate : float):
        
        Send continuously(at a set rate) a certain data supported (in data_types)
        :param name: name of the parameter
        :param type_msg_str: type of the msg, in str ("string", "twist", ...)
        :param get_data_callback: function from the simulator, called to get the latest data
        :param rate: rate in time per seconds
        :return:
        
        raise NotImplementedError()
    """
    def register_msg_callback(self, name: str, dataType:data_type, set_data_callback):
        """
        Function to call preferably when initialising the simulator,
        a callback function from the simulator is called with the arguments provided from the interface depending on the msg type
        Use it to process input from other system outside of sim
        exemple : update_speed, SpeedCmd
        :param callback:
        :param service_type:
        :return:
        """
        raise NotImplementedError()


