from typing import List
from functools import cached_property
import cProfile
import aruco_analysis_enac.settings as settings

class ArucosStorage:
    #manage storage & generation of a ObjectMarked message, for arucos
    def __init__(self,  ref_arucos: settings.Aruco, frame_id="map"):
        """From pose estimation on a reference frame, store and correlate arucos between frames.

        It works only with unique aruco_id for the ones that are fixed to the reference frame.
        It works with non-unique aruco_id for the ones moving in the reference frame.

        Args:
            ref_arucos (settings.Aruco): must be unique id
            frame_id (str, optional): The frame of the table/ground used (for tf.transforms). Defaults to "map".
        """

        self.frame_id = frame_id
        self.ref_arucos = ref_arucos
        if len([aruco.id for aruco in self.ref_arucos]) != len(set([aruco.id for aruco in self.ref_arucos])):
            raise AttributeError("ArucosStorage Error - the same aruco_id has been assigned multiple times on __init__")

    @classmethod
    def from_subset(cls, subset: int):
        """
        Generate an ArucosStorage instance from a subset of arucos.
        """
        
        return cls(settings.get_markers(subset).arucos)

    @cached_property 
    def reference_ids(self) -> List[int]: #this is only settable on init of ArucosStorage
        ids = []
        for aruco in self.ref_arucos:
            ids.append(aruco.id)
        return ids

    """TODO : BELOW CODE IS NOT DONE """
    def aruco_pose_update(self, aruco_id: int, position): #TODO : add position signature
        if not self._correlate(aruco_id, position):
            probablearuco = self._retrieve_live_object(aruco_id, position)
            if not probablearuco:
                self._create_live_object(aruco_id)

    def _correlate(self, aruco_id, position):
        pass
    def _retrieve_live_object(self, aruco_id, position):
        pass
    def _create_live_object(self, aruco_id):
        pass

    


""" 
Instantiation :
Associer un ID avec une position approximative


ID Aruco (non unique) ->(1) ID d'objet ->(2) Position
(1) Tracking - associer à chaque ID un ID d'objet
(2) "Base de données"

IsMoving
"""