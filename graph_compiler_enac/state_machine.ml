let lib = 
"#statemachine/graph_compil library by KirrimK
#library to be used with graph_compil
#version 0.1

class Transition:
    def nothing(self):
        pass
    def accept_by_default(self):
        return True
    def __init__(self, name, destination, on_transition = None, guard = None):
        self.name = name
        self.destination = destination
        if guard is None:
            self.guard = self.accept_by_default
        else:
            self.guard = guard
        if on_transition is None:
            self.on_transition = self.nothing
        else:
            self.on_transition = on_transition
    def __str__(self):
        return f\"Transition({self.name})\"

class State:
    def nothing(self):
        pass
    def __init__(self, name, on_enter = None, on_leave = None, transitions = []):
        self.name = name
        if on_enter is None:
            self.on_enter = self.nothing
        else:
            self.on_enter = on_enter
        if on_enter is None:
            self.on_leave = self.nothing
        else:
            self.on_leave = on_leave
        self.transitions = transitions
    def add_transition(self, transition):
        self.transitions.append(transition)
    def check_transitions(self):
        for transition in self.transitions:
            if transition.guard():
                self.on_leave()
                transition.on_transition()
                return transition.destination
        return None
    def __str__(self):
        return f\"State({self.name})\"

class StateMachine:
    def __init__(self, init_state = None):
        self.state = init_state
        self.state.on_enter()
    
    def check_transitions(self):
        if self.state is not None:
            new_state = self.state.check_transitions()
            if new_state is not None:
                self.state = new_state
                self.state.on_enter()
"