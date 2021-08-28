#!/usr/bin/env python2
import pcl
import pcl_helper
import pcl.pcl_visualization
import numpy as np
import pptk

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def do_smoothing(pc_data, radius):
	print("smoothing ")
	tree = pc_data.make_kdtree()
	mls = pc_data.make_moving_least_squares()
	mls.set_Compute_Normals (True)
	mls.set_polynomial_fit (True)
	mls.set_Search_Method (tree)
	mls.set_search_radius(radius) # Use all neighbors in a radius of 3cm.
	mls_points = mls.process ()
	
	return mls_points

def do_voxel_grid_downsampling(pc_data, leaf_size):
	print("voxel start")
	vox = pc_data.make_voxel_grid_filter()
	vox.set_leaf_size(leaf_size, leaf_size, leaf_size)

	return vox.filter()

def do_pass_through_filtering(pc_data):
	print("passthroguh start")
	passthrough = pc_data.make_passthrough_filter()
	passthrough.set_filter_field_name("z")
	passthrough.set_filter_limits(-35, -20)

	return passthrough.filter()

def do_statistical_outlier_filter(pc_data, mean, std):
	print("do statistical outlier filter")
	fil = pc_data.make_statistical_outlier_filter()
	#gaussian setting
	fil.set_mean_k(mean)
	fil.set_std_dev_mul_thresh(std)

	inliter_filtered = fil.filter() 
	#pcl.save(inliter_filtered,"/home/benlee/Desktop/50_inliers.pcd")
	fil.set_negative(True)
	outlier_filtered = fil.filter()
	#pcl.save(outlier_filtered,"/home/benlee/Desktop/50_ioutliers.pcd")
	return inliter_filtered, outlier_filtered

def read_pcd():
	print("read pcd start")
	profile_np = np.load('/home/benlee/Desktop/0111pcd/pptk_save_2.npy')
	profile_pcd = pcl.load("/home/benlee/Desktop/0111pcd/pptk_save_2.pcd")

	return profile_np, profile_pcd

def visualize(pc_data):
	print("visualize start")
	#centred = pc_data - np.mean(pc_data, 0)
	#pc_centred = pcl.PointCloud()
	#pc_centred.from_array(centred)

	visual = pcl.pcl_visualization.CloudViewing()
	visual.ShowMonochromeCloud(pc_data,b'pc_data')
	v = True
	while v:
		v = not(visual.WasStopped())

def check_inf(pc_data):
	point_size = len(pc_data)
	modified_pc = []
	for i in range(0, point_size):
		temp = pc_data[i]
		changed_arr = np.where(temp == np.inf, 30, temp)

		modified_pc.append(list(changed_arr))
		pc_data[i] = changed_arr 	
	modified_pc = np.array(modified_pc, dtype=np.float32)

	return modified_pc

def filter_remove():
	profile_np, profile_pcd = read_pcd() 
	ori_pc_np = check_inf(profile_np)

	# point cloud numpy, pcd read
	ori_pc = pcl.PointCloud(ori_pc_np) #numpy to pcl

	print("original size :", ori_pc.size)
	pcl.save(ori_pc,"/home/benlee/Desktop/0111pcd/ori_cloud.pcd")
	# original 

	voxel_filtered = do_voxel_grid_downsampling(ori_pc, 0.1) #514,823
	print("voxelfilter size :", voxel_filtered.size)
	pcl.save(voxel_filtered,"/home/benlee/Desktop/0111pcd/voxel_filtered.pcd")
	# voxel filter

	pass_filtered_pc = do_pass_through_filtering(voxel_filtered)       ### works
	pcl.save(pass_filtered_pc,"/home/benlee/Desktop/0111pcd/pass_filtered.pcd")
	print("passfilter size :", pass_filtered_pc.size)
	#pass through

	inlier_pc, outlier_pc =do_statistical_outlier_filter(pass_filtered_pc, 300, 5)  ##statistical filter 
	inlier_pc100, outlier_pc =do_statistical_outlier_filter(pass_filtered_pc, 300, 5)  ##statistical filter 
	pcl.save(inlier_pc,"/home/benlee/Desktop/inlier_pc.pcd")
	pcl.save(inlier_pc100,"/home/benlee/Desktop/inlier_pc100.pcd")
	# statistical outlier

	smoot_pc = do_smoothing(inlier_pc, 1)
	print("smoothing size :", smoot_pc.size)
	pcl.save(smoot_pc,"/home/benlee/Desktop/smoot_pc.pcd")

	# smoothing 
	visualize(smoot_pc)
	final_pc = pcl.PointCloud(smoot_pc) #numpy to pcl
	print("filtering is done")
	
	cloud_new = pcl_helper.pcl_to_ros(final_pc)

	#pointcloud XYZ to ROS PointCloud2 msg
	pub.publish(cloud_new)

	

if __name__=="__main__":
	rospy.init_node('cmm_test', anonymous=True)
	pub = rospy.Publisher("cmm_profile", PointCloud2, queue_size=1)
	filter_remove()
	rospy.spin()