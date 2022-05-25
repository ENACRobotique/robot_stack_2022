#generated using graph_compil v0.1
from statemachine import State, Transition, StateMachine

RecalageC = State("RecalageC")
RecalageB = State("RecalageB")
RecalageA = State("RecalageA")
HasRentreAuBercail = State("HasRentreAuBercail")
Fin = State("Fin")
PushCarre = State("PushCarre")
GoCarre = State("GoCarre")
HasDroppedStatuette = State("HasDroppedStatuette")
AtVitrine = State("AtVitrine")
HasDroppedReplique = State("HasDroppedReplique")
HasTurnedAroundReplique = State("HasTurnedAroundReplique")
HasRecupStatuette = State("HasRecupStatuette")
AtStatuette = State("AtStatuette")
IsRentringAuBercail = State("IsRentringAuBercail")
Init = State("Init", self.on_init)
tr453775270ToIsRentringAuBercail = Transition("tr453775270ToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.quinze_dernieres_secondes)
AtStatuette.add_transition(tr453775270ToIsRentringAuBercail)
HasRecupStatuette.add_transition(tr453775270ToIsRentringAuBercail)
HasTurnedAroundReplique.add_transition(tr453775270ToIsRentringAuBercail)
HasDroppedReplique.add_transition(tr453775270ToIsRentringAuBercail)
AtVitrine.add_transition(tr453775270ToIsRentringAuBercail)
HasDroppedStatuette.add_transition(tr453775270ToIsRentringAuBercail)
GoCarre.add_transition(tr453775270ToIsRentringAuBercail)
PushCarre.add_transition(tr453775270ToIsRentringAuBercail)
tr25103878ToFin = Transition("tr25103878ToFin", Fin, self.tout_flinguer, self.is_fin)
AtStatuette.add_transition(tr25103878ToFin)
HasRecupStatuette.add_transition(tr25103878ToFin)
HasTurnedAroundReplique.add_transition(tr25103878ToFin)
HasDroppedReplique.add_transition(tr25103878ToFin)
AtVitrine.add_transition(tr25103878ToFin)
HasDroppedStatuette.add_transition(tr25103878ToFin)
GoCarre.add_transition(tr25103878ToFin)
PushCarre.add_transition(tr25103878ToFin)
IsRentringAuBercail.add_transition(tr25103878ToFin)
HasRentreAuBercail.add_transition(tr25103878ToFin)
IsRentringAuBercailToHasRentreAuBercail = Transition("IsRentringAuBercailToHasRentreAuBercail", HasRentreAuBercail, self.things_todo_at_bercail, self.is_at_bercail)
IsRentringAuBercail.add_transition(IsRentringAuBercailToHasRentreAuBercail)
InitToAtStatuette = Transition("InitToAtStatuette", AtStatuette, self.go_recup_statuette, self.is_tirette_activee)
Init.add_transition(InitToAtStatuette)
AtStatuetteToHasRecupStatuette = Transition("AtStatuetteToHasRecupStatuette", HasRecupStatuette, self.recup_statuette, self.is_at_statuette)
AtStatuette.add_transition(AtStatuetteToHasRecupStatuette)
tr581141130ToHasTurnedAroundReplique = Transition("tr581141130ToHasTurnedAroundReplique", HasTurnedAroundReplique, self.turn_around_replique, self.has_gotten_statuette)
HasRecupStatuette.add_transition(tr581141130ToHasTurnedAroundReplique)
tr86346170ToHasDroppedReplique = Transition("tr86346170ToHasDroppedReplique", HasDroppedReplique, self.drop_replique, self.has_turned_around_replique)
HasTurnedAroundReplique.add_transition(tr86346170ToHasDroppedReplique)
HasDroppedRepliqueToAtVitrine = Transition("HasDroppedRepliqueToAtVitrine", AtVitrine, self.go_vitrine, self.has_dropped_replique)
HasDroppedReplique.add_transition(HasDroppedRepliqueToAtVitrine)
AtVitrineToHasDroppedStatuette = Transition("AtVitrineToHasDroppedStatuette", HasDroppedStatuette, self.drop_statuette, self.is_at_vitrine)
AtVitrine.add_transition(AtVitrineToHasDroppedStatuette)
HasDroppedStatuetteToRecalageA = Transition("HasDroppedStatuetteToRecalageA", RecalageA, self.recalage_a, self.has_dropped_stat)
HasDroppedStatuette.add_transition(HasDroppedStatuetteToRecalageA)
RecalageAToRecalageB = Transition("RecalageAToRecalageB", RecalageB, self.recalage_b, self.has_recale_a)
RecalageA.add_transition(RecalageAToRecalageB)
RecalageBToRecalageC = Transition("RecalageBToRecalageC", RecalageC, self.recalage_c, self.has_recale_b)
RecalageB.add_transition(RecalageBToRecalageC)
RecalageCToGoCarre = Transition("RecalageCToGoCarre", GoCarre, self.go_carre, self.has_recale_c)
RecalageC.add_transition(RecalageCToGoCarre)
GoCarreToPushCarre = Transition("GoCarreToPushCarre", PushCarre, self.push_carre, self.is_at_carre)
GoCarre.add_transition(GoCarreToPushCarre)
PushCarreToIsRentringAuBercail = Transition("PushCarreToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_waited_some_time)
PushCarre.add_transition(PushCarreToIsRentringAuBercail)
EnacStrat = StateMachine(Init)
def on_init(self):
    pass

def go_bercail(self):
    pass

def quinze_dernieres_secondes(self):
    return True

def tout_flinguer(self):
    pass

def is_fin(self):
    return True

def things_todo_at_bercail(self):
    pass

def is_at_bercail(self):
    return True

def go_recup_statuette(self):
    pass

def is_tirette_activee(self):
    return True

def recup_statuette(self):
    pass

def is_at_statuette(self):
    return True

def turn_around_replique(self):
    pass

def has_gotten_statuette(self):
    return True

def drop_replique(self):
    pass

def has_turned_around_replique(self):
    return True

def go_vitrine(self):
    pass

def has_dropped_replique(self):
    return True

def drop_statuette(self):
    pass

def is_at_vitrine(self):
    return True

def recalage_a(self):
    pass

def has_dropped_stat(self):
    return True

def recalage_b(self):
    pass

def has_recale_a(self):
    return True

def recalage_c(self):
    pass

def has_recale_b(self):
    return True

def go_carre(self):
    pass

def has_recale_c(self):
    return True

def push_carre(self):
    pass

def is_at_carre(self):
    return True

def has_waited_some_time(self):
    return True
