#!/usr/bin/env python2
import pcl
import pcl.pcl_visualization
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pptk

file_path = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data'

def read_pcd():
    print("read pcd")
    #profile_np = np.load(file_path + '/bun000.npy')
    profile_pcd = pcl.load(file_path + '/check1.pcd')
    passfiltered_pcd = do_pass_through_filtering(profile_pcd)
    visualize(passfiltered_pcd)