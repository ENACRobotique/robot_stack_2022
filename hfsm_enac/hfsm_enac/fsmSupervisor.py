from hfsm_enac.hfsm_enac.hfsm import StateMachine, TaskSM

class fsmSupervisor(TaskSM):
    def __init__(self, entryState: StateMachine):
        super().__init__('fsmSupervisor', initial_child_sm=entryState, next_state=None)
        self.currentState = entryState
        self.nextDormantState = None
        self.

    def set_next_task(self, state:TaskSM):
        self.nextDormantState = state

    def _task_on_loop(self):
        pass

    def on_loop(self):
        #manage current task and transition to the next
        self._task_on_loop()
        
        if self.currentState.is_done == False:
            self.currentState.on_loop()
        else:
            if self.currentState.next_state != None:
                self.currentState = self.currentState.next_state
                self.currentState.on_entry()
            elif self.nextDormantState != None:
                self.currentState = self.nextDormantState
                self.nextDormantState = None
            else:
                raise Exception("missing nextDormingState for fsmSupervisor - no state are set !")
        #Check what should be the next task


