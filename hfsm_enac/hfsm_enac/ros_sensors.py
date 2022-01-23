"""
File for sensors that rely on ROS topics
It's possible to make fake sensors in another file
"""

def get_global_costmap():
    raise NotImplementedError()
    return [] #The cost data, in row-major order, starting with (0,0)