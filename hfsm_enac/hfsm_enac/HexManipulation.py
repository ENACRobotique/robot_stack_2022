from hfsm_enac.hfsm_enac.hfsm import State
from hfsm_enac.hfsm_enac.MoveStraight import MoveStraight

class HexManipulation(State):
    def __init__(self, name, store, arm_id:str, arm_action:int):
        super().__init__(name, )
        self.store = store

    def on_entry(self):
        store.update_value(arm_id, arm_action)
        pass

    def on_loop(self):
        if store.get_value(arm_id) == 000000: #TODO : etat de hexagon posé
            self.on_exit()
    def on_exit(self):
        MoveStraight(100,200,1.0) #x,y, theta
        #TODO : faire autre chose que la dépose