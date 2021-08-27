import numpy as np
import pptk
import pcl
import pcl.pcl_visualization

xyz_save_load = np.load('/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/keyence_ros/pcd_files/test_save2.npy')
ori_pc = pcl.PointCloud(xyz_save_load) #numpy to pcl
pcl.save(ori_pc,"/home/benlee/Desktop/ori_cloud.pcd")
print(xyz_save_load)
print(type(xyz_save_load))
print(len(xyz_save_load))
v = pptk.viewer(xyz_save_load)
v.set(point_size=0.001)
