
import string
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_diagnostic_level(level:int):
    if level == 0:
        return DiagnosticStatus.OK
    if level == 1:
        return DiagnosticStatus.WARN
    if level == 2:
        return DiagnosticStatus.ERROR
    if level == 3:
        return DiagnosticStatus.STALE

def send_diagnostic_data(level:int, name:string, message="OK", origin="raspberry"):
    """
    origin = "raspberry" or "stm32"
    """
    #int value can be from 1 to 4 -> 1 OK, 2 ->, 3 -> 4 -> 
    msg = DiagnosticArray()
    reference = DiagnosticStatus()
    reference.level = get_diagnostic_level(level)
    reference.name = name
    reference.message = message
    reference.hardware_id = origin
    reference.values = []
    msg.status = [reference]