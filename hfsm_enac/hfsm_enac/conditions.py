"""
This file should not use ros topics or msg formats directly (for modularity & allow mockable conditions)
"""
from hfsm_enac.hfsm_enac import ros_sensors

def is_area_occupied(pt1,pt2)-> float:
    #TODO : draw a rectangle including pt1 and pt2, check in global_cost_map this area
    # return a float between 0(empty) and 1(occupied) to determine if the area is free or not
    # something like -> max(sensors.get_global_costmap())[pt1:pt2]
    ros_sensors.get_global_costmap()
    raise NotImplementedError()
    return 0

