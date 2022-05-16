#generated using graph_compil v0.1
from statemachine import State, Transition, StateMachine

Bruh = State("Bruh")
Stonks = State("Stonks")
Argh = State("Argh")
StonksToArgh = Transition("StonksToArgh", Argh)
Stonks.add_transition(StonksToArgh)
ArghToStonks = Transition("ArghToStonks", Stonks)
Argh.add_transition(ArghToStonks)
BruhToArgh = Transition("BruhToArgh", Argh)
Bruh.add_transition(BruhToArgh)
bruhMachine = StateMachine(Bruh)

print(Bruh)
print(Stonks)
print(Argh)

print("-----")

print(bruhMachine)
bruhMachine.check_transitions()
print(bruhMachine)
bruhMachine.check_transitions()
print(bruhMachine)
bruhMachine.check_transitions()
print(bruhMachine)