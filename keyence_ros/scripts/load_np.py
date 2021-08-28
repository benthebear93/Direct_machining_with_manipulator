import numpy as np
import pptk
import pcl
import pcl.pcl_visualization

xyz_save_load = np.load('/home/benlee/catkin_ws/src/pptk_save_reduce2.npy')
ori_pc = pcl.PointCloud(xyz_save_load) #numpy to pcl
pcl.save(ori_pc,"/home/benlee/Desktop/pptk_save_reduce2.pcd")
print(xyz_save_load)
print(type(xyz_save_load[0][0]))
print(len(xyz_save_load))
v = pptk.viewer(xyz_save_load)
v.set(point_size=0.00001)
