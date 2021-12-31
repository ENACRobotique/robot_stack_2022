import logging

from hfsm_enac.hfsm_enac import ros_sensors

from hfsm_enac.hfsm_enac.hfsm import StateMachine, TaskSM
import hfsm_enac.hfsm_enac.ros_actuators as actuators
import hfsm_enac.hfsm_enac.conditions as conditions


def has_overshoot_y(isComingLeft, maxPosition):
    #ros_sensors.get_current_position
    if isComingLeft: #and ros_sensors.current_position > maxPosition
        return True
    elif not isComingLeft: #and ros_sensors.current_position < maxPosition
        return True
    else:
        return False

class CheckDigSquare(TaskSM):
    def __init__(self, name, initialPosition, initialOrientation, endMaxPosition, nextSquare = None):
        super().__init__(name, )
        self.isComingLeft = True if initialPosition.x < endMaxPosition.x else False
        self.endMaxPosition = endMaxPosition
        self.nextSquare = nextSquare

    def on_entry(self):
        pass
        #TODO : retracter tous les actuateurs sauf celui du bras, et le sortir  - actuators.retract_perimeters_actuator()

    def on_loop(self):
        #if sensors.courant == valeur plausible : faire des choses avec, & on_exit
        if conditions.has_overshoot_y(self.isComingLeft, self.endMaxPosition):
            logging.info(f"abnormal exit from state machine {super().name} | robot was checking the dig squares but overshooted one ! "
                         f"Comportment not taken into account for score")
            self.on_exit()
    def on_exit(self):
        if self.nextSquare == None:
            actuators.retract_perimeters_actuator()
