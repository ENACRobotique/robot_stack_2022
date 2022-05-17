let lib = 
"#statemachine/graph_compil library by KirrimK
#library to be used with graph_compil
#version 0.1

class Transition:
    def nothing(self):
        print(f\"{self.name}: nothing\")
    def accept_by_default(self):
        print(f\"{self.name}: accept_by_default\")
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
    def on_enter_default(self):
        print(f\"{self.name}: on_enter_default\")
    def on_leave_default(self):
        print(f\"{self.name}: on_leave_default\")
    def __init__(self, name, on_enter = None, on_leave = None, transitions = []):
        self.name = name
        if on_enter is None:
            self.on_enter = self.on_enter_default
        else:
            self.on_enter = on_enter
        if on_leave is None:
            self.on_leave = self.on_leave_default
        else:
            self.on_leave = on_leave
        self.transitions = transitions.copy()
    def add_transition(self, transition):
        self.transitions.append(transition)
    def check_transitions(self):
        for transition in self.transitions:
            if transition.guard():
                transition.on_transition()
                return transition.destination
        return None
    def __str__(self):
        str_trs = \"\"
        for t in self.transitions:
            str_trs += str(t)+\" \"
        return f\"State({self.name}, [{str_trs}])\"

class StateMachine:
    def __init__(self, init_state = None):
        self.state = init_state
        self.is_started = False
    
    def start(self):
        if not self.is_started:
            self.state.on_enter()
            self.is_started = True
    
    def check_transitions(self):
        if self.state is not None and self.is_started:
            new_state = self.state.check_transitions()
            if new_state is not None:
                if new_state != self.state:
                    self.state.on_leave()
                    new_state.on_enter()
                self.state = new_state
    
    def __str__(self):
        return f\"StateMachine({str(self.state)})\"
"