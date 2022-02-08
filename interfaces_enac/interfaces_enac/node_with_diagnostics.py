import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class status():
    def __init__(self, component_name, messages = {Enum: ''}, key_values = {}) -> None:
        pass
class NodeDiagnostics(Node):   
    def __init__(self, name=''):
        super.__init__(name)
        self.hardware_id = name #TODO : add namespace
        self.status = {}

        self.diagnostics = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
    

    def declare_diagnostic_component(self, component_name, messages = {Enum: ''}, key_values = {}):
        self.status[component_name] = {'messages': messages, 'key_values': key_values}
        """
        class reference_detection(Enum):
            NONE = 0
            ONE = 1
            TWO = 2
            THREE = 3
        declare_diagnostic_component(
            'aruco_references_detection',
            {'NONE': 'No aruco has been detected',
            'ONE' : 'one reference aruco has been detected',
                [....]
            },
            key_values= {
                {'expected_arucos_references' : aruco_storage.reference_ids,
                'detected_arucos_references' : analysis_aruco.detected_references, }
            }
        )
        """
    def update_diagnostics(self, component_name: str, level: int, message_enum: Enum):
        #self.diagnostics.publish(msg)
        pass
    def __int_to_level(self, level_int):
        """[summary]

        Args:
            level_int ([type]): [description]

        Returns:
            [type]: [description]

        byte OK=0
        byte WARN=1
        byte ERROR=2
        byte STALE=3
        """
        if level_int == 0:
            return DiagnosticStatus.OK
        else:
            return DiagnosticStatus.ERROR

