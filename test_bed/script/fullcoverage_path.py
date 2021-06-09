#!/usr/bin/env python2
import pcl
import pcl.pcl_visualization
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import cv2
import matplotlib.pyplot as plt

file_path = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/test_bed/pcd_data'
def read_pcd():
    print("read pcd")
    #profile_np = np.load(file_path + '/bun000.npy')
    profile_pcd = pcl.load(file_path + '/cluster10.pcd')
    print(profile_pcd.size)
    x = []
    y = []
    for i in range(profile_pcd.size):
        x.append(profile_pcd[i][0])
        y.append(profile_pcd[i][1])
    print("x max:", max(x))
    print("y max:", max(y))
    print("x difference", (max(x)-min(x))*1000)
    print("x min:", min(x))
    print("y min:", min(y))
    print("y difference", (max(y)-min(y))*1000)
    visualize(profile_pcd)

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

if __name__=="__main__":
    read_pcd()