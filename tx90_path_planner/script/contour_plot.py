#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
from std_msgs.msg import Int32MultiArray
from tx90_path_planner.msg import boundary
from tx90_path_planner.msg import scan_path

import numpy as np
from matplotlib.patches import Polygon

from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from matplotlib.path import Path
from std_msgs.msg import Int32
EXTEND_AREA = 10.0

def simple_plot(data_x, data_y, min_x, max_x, min_y, max_y):
    fig, ax = plt.subplots()
    plt.setp(ax.spines.values(), linewidth=1.7)
    ax.set_title("Workpiece contour", fontsize=17)
    ax.set_xlabel('x (mm)', fontsize = 16)
    ax.set_ylabel('y (mm)', fontsize = 16)

    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())

    ax.xaxis.set_ticks_position('both')
    ax.yaxis.set_ticks_position('both')
    #ax.yaxis.set_tick_params(which='minor',direction="in")
    ax.tick_params(axis="x", direction="in", which='major', labelsize=13, width=2)
    ax.tick_params(axis="x", direction="in", which='minor', labelsize=13)

    ax.tick_params(axis="y", direction="in", which='major', labelsize=13, width=2)
    ax.tick_params(axis="y", direction="in", which='minor', labelsize=13)

    ax.set_axisbelow(True)
    extra_space = 25
    ax.set_xlim(xmin=min_x-extra_space, xmax=max_x+extra_space)
    ax.set_ylim(ymin=min_y-extra_space, ymax=max_y+extra_space)
    ax.scatter(data_x, data_y, label='workpiece contour')
    ax.grid(color="#A2A6AB")
    plt.legend(loc='upper right', ncol=1)
    plt.show()

def gen_grid_map(ox, oy, xy_resolution):

	min_x = round(min(ox) - EXTEND_AREA / 2.0) 
	min_y = round(min(oy) - EXTEND_AREA / 2.0) 
	max_x = round(max(ox) + EXTEND_AREA / 2.0) 
	max_y = round(max(oy) + EXTEND_AREA / 2.0) 

	xw = int(round((max_x - min_x) / xy_resolution)) 
	yw = int(round((max_y - min_y) / xy_resolution)) 

	print("min_x :", min_x, "max_x :", max_x,"diff :", round((max_x - min_x)))
	print("min_y :", min_y, "max_y :", max_y,"diff :", round((max_y - min_y)))
	print("max grid map is x: ", xw, " y: ", yw) 
	grid_map = np.zeros((yw, xw)) / 2

	return grid_map, min_x, max_x, min_y, max_y, xy_resolution

def get_boundary(data):

    sensor_x_length = 32
    xy_resolution = 11 # sensor parameter

    x_data = data.boundary_x
    y_data = data.boundary_y 
    x_data = [ int(x * 1000) for x in x_data]
    y_data = [ int(y * 1000) for y in y_data]
    # x_data = np.sort(x_data)
    # y_data = np.sort(y_data)
    print("x_data: ", x_data, "y_data: ", y_data)
    grid_resolution = int(round(sensor_x_length/xy_resolution))
    grid_map, min_x, max_x, min_y, max_y, xy_resolution = gen_grid_map(x_data, y_data, xy_resolution)
    simple_plot(x_data, y_data, min_x, max_x, min_y, max_y)
    grid_x = [(x - min_x)/xy_resolution for x in x_data]
    grid_y = [(y - min_y)/xy_resolution for y in y_data] #expand scanning area in case
    print("scan grid map is x: ", round(max(grid_x)-min(grid_x)), " y: ", round(max(grid_y)-min(grid_y))) 
    print("scan grid map is x: ", (max_x - min_x)/xy_resolution, " y: ",(max_y-min_y)/xy_resolution) 

def main():

	rospy.init_node('contour_plot',anonymous=True)
	sub_y = rospy.Subscriber('boundary', boundary, get_boundary)
	rospy.spin()

if __name__ == '__main__': 
	main()