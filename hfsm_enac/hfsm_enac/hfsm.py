


class StateMachine():
    def __init__(self, name, next_state=None):
        """
        nextState : another StateMachine
        exitConditions : functions that can be called
        """
        self.name = name
        self.next_state=None
        self.is_done = False

    def on_entry(self):
        raise NotImplementedError()
    def on_loop(self):
        #loop should manage its exit
        raise NotImplementedError()
    def on_exit(self):
        """
        For multiple exit state possible, overwrite this function for conditions (but keep self.is_done to true)
        """
        self.is_done = True

class TaskSM(StateMachine):
    def __init__(self, name, initial_child_sm=None, next_state=None):
        super().__init__(name, next_state=next_state)
        self.cur_child_sm = initial_child_sm

    def task_loop(self):
        raise NotImplementedError()
    def on_loop(self):
        self.cur_child_sm.on_loop()
        self.task_loop()
        if self.cur_child_sm.is_done:



