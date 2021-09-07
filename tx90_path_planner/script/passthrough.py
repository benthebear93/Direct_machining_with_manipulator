import pcl
import numpy as np
import matplotlib.pyplot as plt
from random import randint

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):

    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

filepath = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner/pcd_data'
cloud = pcl.load(filepath + "/surface.pcd") # Deprecated; use pcl.load instead.
filter_axis = 'z'
axis_min = 0.24
axis_max = 0.3
cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
pcl.save(cloud, filepath + "/surface2.pcd")