import logging

from hfsm_enac.hfsm_enac.hfsm import State

class fsmSupervisor(State):
    def __init__(self, entry_child_sm: State):
        super().__init__('fsmSupervisor', initial_child_sm=entry_child_sm, next_state=None)

    def on_entry(self):
        pass

    def _task_loop(self):
        #manage case self.cur_child_sm exit but not done
        pass

    def _on_empty_next_child(self):
        # manage case self.cur_child_sm exit but not done
        pass

    def on_exit(self):
        logging.error("fsmSupervisor state exited ! | behaviour not taken into account")
        self.is_done = True

