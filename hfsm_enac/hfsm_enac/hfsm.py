class State():
    """
    Implementation of a state, manage its transition and the one of its current child
    """
    def __init__(self, name, next_state=None, initial_child_sm=None):
        super().__init__(name, next_state=next_state)
        self.cur_child_sm = initial_child_sm
        self.next_state=next_state
        self.is_done = False

    def on_entry(self):
        raise NotImplementedError()

    def _task_loop(self):
        #this function should call on_exit when this state is over
        raise NotImplementedError()

    def _on_empty_next_child(self):
        pass
        #self.cur_child_sm = ...

    def on_loop(self):
        if self.cur_child_sm is not None:
            if not self.cur_child_sm.is_done:
                self.cur_child_sm.on_loop()
            else:
                if self.cur_child_sm.next_state is not None:
                    self.cur_child_sm = self.cur_child_sm.next_state
                    self.cur_child_sm.on_entry()
                else:
                    self.cur_child_sm = None
                    #raise Exception("missing next_child_state for task SM - no state are set !")

        if self.cur_child_sm is None: #not an if/else in case it become None during this loop
            self._on_empty_next_child()

        self._task_loop()

    def on_exit(self):
        """
        For multiple exit state possible, overwrite this function for conditions (but keep self.is_done to true)
        """
        self.is_done = True

class StoreActuator():
    def __init__(self) -> None:
        self.on_update_value = None
        self.values = {}
        pass
    def connect_to_ros(self, on_update_value):
        self.on_update_value = on_update_value

    def update_value(self, id, position:int):
        try:
            on_update_value()
        except Exception as e:
            print(e)
            print("it's possible that the on_update_value of the store has not been set due to not calling connect_to_ros")

    def get_value(self, key):
        if self.values[key] == None:
            print("error missing value")
        else:
            return self.values[key]