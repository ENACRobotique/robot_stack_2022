import logging
import rclpy.logging

ros_stream_handler = RosStreamHandler()
ros_stream_handler.setLevel(logging.DEBUG)
logger.addHandler(ros_stream_handler)

import rclpy
from hfsm_enac.hfsm_enac.fsmSupervisor import fsmSupervisor
from hfsm_enac.hfsm_enac.listTasks import wait_departure_signal

class fsmNode(rclpy.Node):
    def __init__(self) -> None:
        super().__init__('fsmNode')
        #subscribe to peripherals
        #publish goal_pose
        #publish to peripherals
    def move_actuator(id:str, position:int):
        pass #publish to peripherals

    #fsmSupervisor(wait_departure_signal)
def main():
    rclpy.init()
    node = fsmNode()
    logging.StreamHandler().setStream(rclpy.logging.get_logger())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()