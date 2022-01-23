from typing import List

class ArucosStorage:
    #manage storage & generation of a ObjectMarked message, for arucos
    def __init__(self,  arucos, frame_id="map"):
        pass
    
    def get_reference_ids() -> List[int]:
        return [42]

""" 
Instantiation :
Associer un ID avec une position approximative


ID Aruco (non unique) ->(1) ID d'objet ->(2) Position
(1) Tracking - associer à chaque ID un ID d'objet
(2) "Base de données"

IsMoving
"""