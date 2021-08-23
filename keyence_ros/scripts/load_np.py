import numpy as np
import pptk

xyz_save_load = np.load('/home/benlee/catkin_ws/src/pptk_save.npy')
print(xyz_save_load)
print(type(xyz_save_load))
print(len(xyz_save_load))
v = pptk.viewer(xyz_save_load)
v.set(point_size=0.000001)