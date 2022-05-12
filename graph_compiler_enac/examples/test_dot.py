#generated using graph_compil v0.1
from statemachine import State, Transition, StateMachine

#---partie ajoutée manuellement ci-dessous
class TestNode:
    def __init__(self):
        pass
    
    def on_enter(self):
        print("on_enter")
    
    def on_enter_(self):
        print("on_enter_")

    def on_leave(self):
        print("on_leave")
    
    def on_leave_(self):
        print("on_leave_")
    
    def on_transition(self):
        print("on_transition")

    def guard(self):
        print("guard")
        return True

bruh = TestNode()

#---

Init = State("Init", bruh.on_enter, bruh.on_leave)
Outhome = State("Outhome", bruh.on_enter_, bruh.on_leave_)
End = State("End")
InitOuthomeToEnd = Transition("InitOuthomeToEnd", End, bruh.on_transition, bruh.guard)
Init.add_transition(InitOuthomeToEnd)
Outhome.add_transition(InitOuthomeToEnd)
test = StateMachine(Init)

#--- partie ajoutée manuellement ci-dessous

bruh.test = test

print(bruh.test.state)
bruh.test.check_transitions()
print(bruh.test.state)