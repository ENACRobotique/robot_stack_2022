import hfsm_enac.hfsm_enac.CheckDigSquare as CheckDigSquare

wait_departure_signal = None

carreGauche10 = CheckDigSquare.CheckDigSquare('carreGauche10', 500, 0, 550)
carreGauche9 = CheckDigSquare.CheckDigSquare('carreGauche9', 450, 0, 500, carreGauche10)
TasksOrder = [

]