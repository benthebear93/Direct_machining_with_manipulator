# import pcl
# import pcl.pcl_visualization
import numpy as np
# from plyfile import PlyData, PlyElement
# import pptk
import open3d as o3d

file_path = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data'

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(file_path+ "/check1.pcd")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])