#generated using graph_compil v0.1
from statemachine import State, Transition, StateMachine

ProchainCarre = State("ProchainCarre")
LireCarre = State("LireCarre")
ColleAuMur = State("ColleAuMur")
HasRentreAuBercail = State("HasRentreAuBercail")
PousserSiBesoin = State("PousserSiBesoin")
QuitterMur = State("QuitterMur")
FinCarres = State("FinCarres")
DevantCarres = State("DevantCarres")
FinGalerie = State("FinGalerie")
HasDroppedBleuGalerie = State("HasDroppedBleuGalerie")
DevantGalerieBleuRetourne = State("DevantGalerieBleuRetourne")
HasDroppedVertGalerie = State("HasDroppedVertGalerie")
DevantGalerieVertRetourne = State("DevantGalerieVertRetourne")
HasDroppedRougeGalerie = State("HasDroppedRougeGalerie")
DevantGalerieRougeRetourne = State("DevantGalerieRougeRetourne")
HasRecupPaletBleu = State("HasRecupPaletBleu")
DevantPaletBleu = State("DevantPaletBleu")
HasRecupPaletVertAndStored = State("HasRecupPaletVertAndStored")
DevantPaletVert = State("DevantPaletVert")
RougeInBackHand = State("RougeInBackHand")
HasRecupPaletRougeAndStored = State("HasRecupPaletRougeAndStored")
DevantPaletRouge = State("DevantPaletRouge")
HasDroppedStatuette = State("HasDroppedStatuette")
AtVitrine = State("AtVitrine")
HasDroppedReplique = State("HasDroppedReplique")
HasTurnedAroundReplique = State("HasTurnedAroundReplique")
HasRecupStatuette = State("HasRecupStatuette")
AtStatuette = State("AtStatuette")
IsRentringAuBercail = State("IsRentringAuBercail")
Init = State("Init", self.on_init)
tr139301719ToIsRentringAuBercail = Transition("tr139301719ToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.quinze_dernieres_secondes)
AtStatuette.add_transition(tr139301719ToIsRentringAuBercail)
HasRecupStatuette.add_transition(tr139301719ToIsRentringAuBercail)
HasTurnedAroundReplique.add_transition(tr139301719ToIsRentringAuBercail)
HasDroppedReplique.add_transition(tr139301719ToIsRentringAuBercail)
AtVitrine.add_transition(tr139301719ToIsRentringAuBercail)
HasDroppedStatuette.add_transition(tr139301719ToIsRentringAuBercail)
DevantPaletRouge.add_transition(tr139301719ToIsRentringAuBercail)
HasRecupPaletRougeAndStored.add_transition(tr139301719ToIsRentringAuBercail)
RougeInBackHand.add_transition(tr139301719ToIsRentringAuBercail)
DevantPaletVert.add_transition(tr139301719ToIsRentringAuBercail)
HasRecupPaletVertAndStored.add_transition(tr139301719ToIsRentringAuBercail)
DevantPaletBleu.add_transition(tr139301719ToIsRentringAuBercail)
HasRecupPaletBleu.add_transition(tr139301719ToIsRentringAuBercail)
DevantGalerieRougeRetourne.add_transition(tr139301719ToIsRentringAuBercail)
HasDroppedRougeGalerie.add_transition(tr139301719ToIsRentringAuBercail)
DevantGalerieVertRetourne.add_transition(tr139301719ToIsRentringAuBercail)
HasDroppedVertGalerie.add_transition(tr139301719ToIsRentringAuBercail)
DevantGalerieBleuRetourne.add_transition(tr139301719ToIsRentringAuBercail)
HasDroppedBleuGalerie.add_transition(tr139301719ToIsRentringAuBercail)
FinGalerie.add_transition(tr139301719ToIsRentringAuBercail)
DevantCarres.add_transition(tr139301719ToIsRentringAuBercail)
FinCarres.add_transition(tr139301719ToIsRentringAuBercail)
PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, self.quitter_mur_rentrer_poelon, self.quinze_dernieres_secondes)
PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
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
HasDroppedStatuetteToDevantPaletRouge = Transition("HasDroppedStatuetteToDevantPaletRouge", DevantPaletRouge, self.go_palet_rouge, self.is_prio_galerie)
HasDroppedStatuette.add_transition(HasDroppedStatuetteToDevantPaletRouge)
tr443085219ToHasRecupPaletRougeAndStored = Transition("tr443085219ToHasRecupPaletRougeAndStored", HasRecupPaletRougeAndStored, self.recup_rouge_stocker, self.is_at_palet_rouge)
DevantPaletRouge.add_transition(tr443085219ToHasRecupPaletRougeAndStored)
tr224422110ToRougeInBackHand = Transition("tr224422110ToRougeInBackHand", RougeInBackHand, self.put_back_rouge, self.has_stored_rouge)
HasRecupPaletRougeAndStored.add_transition(tr224422110ToRougeInBackHand)
RougeInBackHandToDevantPaletVert = Transition("RougeInBackHandToDevantPaletVert", DevantPaletVert, self.go_palet_vert, self.has_backhand_rouge)
RougeInBackHand.add_transition(RougeInBackHandToDevantPaletVert)
tr487924451ToHasRecupPaletVertAndStored = Transition("tr487924451ToHasRecupPaletVertAndStored", HasRecupPaletVertAndStored, self.recup_vert_stocker, self.is_at_palet_vert)
DevantPaletVert.add_transition(tr487924451ToHasRecupPaletVertAndStored)
tr226431985ToDevantPaletBleu = Transition("tr226431985ToDevantPaletBleu", DevantPaletBleu, self.go_palet_bleu, self.has_stored_vert)
HasRecupPaletVertAndStored.add_transition(tr226431985ToDevantPaletBleu)
DevantPaletBleuToHasRecupPaletBleu = Transition("DevantPaletBleuToHasRecupPaletBleu", HasRecupPaletBleu, self.recup_bleu, self.is_at_palet_bleu)
DevantPaletBleu.add_transition(DevantPaletBleuToHasRecupPaletBleu)
tr298394497ToDevantGalerieRougeRetourne = Transition("tr298394497ToDevantGalerieRougeRetourne", DevantGalerieRougeRetourne, self.go_galerie_rouge, self.has_recup_bleu)
HasRecupPaletBleu.add_transition(tr298394497ToDevantGalerieRougeRetourne)
tr156138794ToHasDroppedRougeGalerie = Transition("tr156138794ToHasDroppedRougeGalerie", HasDroppedRougeGalerie, self.depot_rouge_arriere, self.is_at_galerie_rouge_retourne)
DevantGalerieRougeRetourne.add_transition(tr156138794ToHasDroppedRougeGalerie)
tr1003384989ToDevantGalerieVertRetourne = Transition("tr1003384989ToDevantGalerieVertRetourne", DevantGalerieVertRetourne, self.go_galerie_vert, self.has_dropped_rouge)
HasDroppedRougeGalerie.add_transition(tr1003384989ToDevantGalerieVertRetourne)
tr168523427ToHasDroppedVertGalerie = Transition("tr168523427ToHasDroppedVertGalerie", HasDroppedVertGalerie, self.destore_drop_vert_arriere, self.is_at_galerie_vert_retourne)
DevantGalerieVertRetourne.add_transition(tr168523427ToHasDroppedVertGalerie)
tr210589313ToDevantGalerieBleuRetourne = Transition("tr210589313ToDevantGalerieBleuRetourne", DevantGalerieBleuRetourne, self.store_bleu_from_front_hand_and_go_galerie_bleu, self.has_dropped_vert)
HasDroppedVertGalerie.add_transition(tr210589313ToDevantGalerieBleuRetourne)
tr432180998ToHasDroppedBleuGalerie = Transition("tr432180998ToHasDroppedBleuGalerie", HasDroppedBleuGalerie, self.destore_drop_bleu_arriere, self.is_at_galerie_bleu)
DevantGalerieBleuRetourne.add_transition(tr432180998ToHasDroppedBleuGalerie)
HasDroppedBleuGalerieToFinGalerie = Transition("HasDroppedBleuGalerieToFinGalerie", FinGalerie, self.do_nothing, self.has_dropped_bleu)
HasDroppedBleuGalerie.add_transition(HasDroppedBleuGalerieToFinGalerie)
FinGalerieToDevantCarres = Transition("FinGalerieToDevantCarres", DevantCarres, self.go_carres, self.pas_deja_fait_carres)
FinGalerie.add_transition(FinGalerieToDevantCarres)
FinGalerieToIsRentringAuBercail = Transition("FinGalerieToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_deja_fait_carres)
FinGalerie.add_transition(FinGalerieToIsRentringAuBercail)
HasDroppedStatuetteToDevantCarres = Transition("HasDroppedStatuetteToDevantCarres", DevantCarres, self.go_carres, self.is_prio_carres)
HasDroppedStatuette.add_transition(HasDroppedStatuetteToDevantCarres)
DevantCarresToColleAuMur = Transition("DevantCarresToColleAuMur", ColleAuMur, self.se_coller_au_mur_deployer_poelon, self.is_devant_carres)
DevantCarres.add_transition(DevantCarresToColleAuMur)
ColleAuMurToLireCarre = Transition("ColleAuMurToLireCarre", LireCarre, self.lire_carre_si_besoin, self.is_au_mur)
ColleAuMur.add_transition(ColleAuMurToLireCarre)
LireCarreToPousserSiBesoin = Transition("LireCarreToPousserSiBesoin", PousserSiBesoin, self.pousser_si_besoin, self.has_lu_carre_si_besoin)
LireCarre.add_transition(LireCarreToPousserSiBesoin)
PousserSiBesoinToQuitterMur = Transition("PousserSiBesoinToQuitterMur", QuitterMur, self.quitter_mur_rentrer_poelon, self.tous_carres_lus)
PousserSiBesoin.add_transition(PousserSiBesoinToQuitterMur)
PousserSiBesoinToProchainCarre = Transition("PousserSiBesoinToProchainCarre", ProchainCarre, self.go_prochain_carre, self.has_checked_carre)
PousserSiBesoin.add_transition(PousserSiBesoinToProchainCarre)
ProchainCarreToLireCarre = Transition("ProchainCarreToLireCarre", LireCarre, self.lire_carre_si_besoin, self.is_at_prochain)
ProchainCarre.add_transition(ProchainCarreToLireCarre)
QuitterMurToFinCarres = Transition("QuitterMurToFinCarres", FinCarres, self.do_nothing, self.has_quitte_mur)
QuitterMur.add_transition(QuitterMurToFinCarres)
FinCarresToIsRentringAuBercail = Transition("FinCarresToIsRentringAuBercail", IsRentringAuBercail, self.go_bercail, self.has_deja_fait_galerie)
FinCarres.add_transition(FinCarresToIsRentringAuBercail)
FinCarresToDevantPaletRouge = Transition("FinCarresToDevantPaletRouge", DevantPaletRouge, self.go_palet_rouge_depuis_mur, self.pas_deja_fait_galerie)
FinCarres.add_transition(FinCarresToDevantPaletRouge)
EnacStrat = StateMachine(Init)
def on_init(self):
    pass

def go_bercail(self):
    pass

def quinze_dernieres_secondes(self):
    return True

def quitter_mur_rentrer_poelon(self):
    pass

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

def go_palet_rouge(self):
    pass

def is_prio_galerie(self):
    return True

def recup_rouge_stocker(self):
    pass

def is_at_palet_rouge(self):
    return True

def put_back_rouge(self):
    pass

def has_stored_rouge(self):
    return True

def go_palet_vert(self):
    pass

def has_backhand_rouge(self):
    return True

def recup_vert_stocker(self):
    pass

def is_at_palet_vert(self):
    return True

def go_palet_bleu(self):
    pass

def has_stored_vert(self):
    return True

def recup_bleu(self):
    pass

def is_at_palet_bleu(self):
    return True

def go_galerie_rouge(self):
    pass

def has_recup_bleu(self):
    return True

def depot_rouge_arriere(self):
    pass

def is_at_galerie_rouge_retourne(self):
    return True

def go_galerie_vert(self):
    pass

def has_dropped_rouge(self):
    return True

def destore_drop_vert_arriere(self):
    pass

def is_at_galerie_vert_retourne(self):
    return True

def store_bleu_from_front_hand_and_go_galerie_bleu(self):
    pass

def has_dropped_vert(self):
    return True

def destore_drop_bleu_arriere(self):
    pass

def is_at_galerie_bleu(self):
    return True

def do_nothing(self):
    pass

def has_dropped_bleu(self):
    return True

def go_carres(self):
    pass

def pas_deja_fait_carres(self):
    return True

def has_deja_fait_carres(self):
    return True

def is_prio_carres(self):
    return True

def se_coller_au_mur_deployer_poelon(self):
    pass

def is_devant_carres(self):
    return True

def lire_carre_si_besoin(self):
    pass

def is_au_mur(self):
    return True

def pousser_si_besoin(self):
    pass

def has_lu_carre_si_besoin(self):
    return True

def tous_carres_lus(self):
    return True

def go_prochain_carre(self):
    pass

def has_checked_carre(self):
    return True

def is_at_prochain(self):
    return True

def has_quitte_mur(self):
    return True

def has_deja_fait_galerie(self):
    return True

def go_palet_rouge_depuis_mur(self):
    pass

def pas_deja_fait_galerie(self):
    return True
