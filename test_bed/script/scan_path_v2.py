#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
from std_msgs.msg import Int32MultiArray
from test_bed.msg import boundary
from test_bed.msg import scan_path

import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib.path import Path
from std_msgs.msg import Int32
EXTEND_AREA = 100.0

class ScanPath():
	def __init__(self, xy_resolution, sensor_x_length):
		self.xy_resolution = xy_resolution #11mm for one grid
		self.sensor_x_length = sensor_x_length #32mm 
		self.path_pub = rospy.Publisher('path', scan_path, queue_size=10)
		
	def planning(self, x_start, y_start, x_end, y_end, yw, sweep_dir, up_down, grid_map):
		line_counter = 1
		x_pos = x_start
		y_pos = y_start
		#print("x_pos", x_pos, "y_pos", y_pos)
		px = []
		py = []
		flag = 0
		while True:
			if y_pos < y_end:
				n_x_pos = x_pos + sweep_dir
				n_y_pos = y_pos
				if grid_map[n_y_pos][n_x_pos] ==True:
					px.append(n_x_pos)
					py.append(n_y_pos)
					x_pos = n_x_pos
					y_pos = n_y_pos
				else:
					x_pos, y_pos, sweep_dir = find_sweep_dir(x_pos, y_pos, sweep_dir, up_down)
					line_counter+=1
					px.append(x_pos)
					py.append(y_pos)
				if y_pos == y_end and x_pos == x_end:
					break
			else:
				n_x_pos = x_pos + sweep_dir
				if sweep_dir > 0:
					if n_x_pos > x_end:
						break
				else:
					if n_x_pos < x_start:
						break
				x_pos = n_x_pos
				px.append(x_pos)
				py.append(y_pos)
		print("line :", line_counter)
		xy_res = np.array(grid_map).shape #x y resolution
		plt.imshow(grid_map, label='occupancy map')
		plt.plot(px, py, '-k', label='scan path')
		plt.gca().set_xticks(np.arange(0, xy_res[1], 1)) 
		plt.gca().set_yticks(np.arange(0, xy_res[0], 1)) 
		plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
		plt.grid(True, which="major", color="w", linewidth=0.6, alpha=0.5) # plot grid
		plt.legend(loc='best')
		plt.subplot(111)
		plt.show()
		print("planning start")
		return px, py

	def get_boundary(self, data):
		x_data = data.boundary_x
		y_data = data.boundary_y 
		x_data = [ int(x * 1000) for x in x_data]
		y_data = [ int(y * 1000) for y in y_data]
		# print(x_data)
		# print(y_data)
		up_down_size = int(round(self.sensor_x_length/self.xy_resolution))
		grid_map, min_x, max_x, min_y, max_y, self.xy_resolution, xw, yw = gen_grid_map(x_data, y_data, self.xy_resolution)
		grid_x = [(x - min_x)/self.xy_resolution for x in x_data]
		grid_y = [(y - min_y)/self.xy_resolution for y in y_data] #adjust 

		corners = select_scan_area(grid_x, grid_y)
		x_start = int(round(corners[1][0]))
		y_start = int(round(corners[1][1]))
		x_end = int(round(corners[2][0]))
		y_end = int(round(corners[2][1]))

		#print(x_start, y_start, x_end, y_end)
		for i in range(y_start,y_end+1):
			for j in range(x_start, x_end+1):
				grid_map[i][j] = 1

		from_upper, sweep_dir, up_down = sweep_updown(x_start, x_end, y_start, y_end, up_down_size)
		px , py = self.planning(x_start, y_start, x_end, y_end,yw, sweep_dir, up_down, grid_map)
		px = [ (x * self.xy_resolution+min_x)/1000 for x in px]
		py = [ (y * self.xy_resolution+min_y)/1000 for y in py]
		path = scan_path() 
		path.path_x = px
		path.path_y = py
		self.path_pub.publish(path)
		#print(px)
		#print(py)

def find_sweep_dir(x_pos, y_pos, sweep_dir, up_down):
	print("changing line")
	sweep_dir *= -1
	x_pos = x_pos
	y_pos = y_pos + up_down

	return  x_pos, y_pos, sweep_dir

def sweep_updown(x_start, x_end, y_start, y_end, up_down_size):
	if y_start < y_end:
		from_upper = True
		up_down = up_down_size
	else: 
		from_upper = False
		up_down =  -up_down_size
	if x_start < x_end:
		sweep_dir = 1
	else:
		sweep_dir = -1
	return from_upper, sweep_dir, up_down

def select_scan_area(grid_x, grid_y):

	left_top = [min(grid_x), min(grid_y)]
	right_top = [max(grid_x), min(grid_y)]
	left_bottom = [min(grid_x), max(grid_y)]
	right_bottom = [max(grid_x), max(grid_y)]
	corners = [left_bottom, left_top, right_bottom, right_top]
	return  corners

def gen_grid_map(ox, oy, xy_resolution):

	min_x = round(min(ox) - EXTEND_AREA / 2.0) 
	min_y = round(min(oy) - EXTEND_AREA / 2.0) 
	max_x = round(max(ox) + EXTEND_AREA / 2.0) 
	max_y = round(max(oy) + EXTEND_AREA / 2.0) 

	xw = int(round((max_x - min_x) / xy_resolution)) 
	yw = int(round((max_y - min_y) / xy_resolution)) 

	print("min_x :", min_x, "max_x :", max_x,"diff :", round((max_x - min_x)))
	print("min_y :", min_y, "max_y :", max_y,"diff :", round((max_y - min_y)))
	print("The grid map is ", xw, "x", yw, ".") 
	grid_map = np.zeros((yw, xw)) / 2

	return grid_map, min_x, max_x, min_y, max_y, xy_resolution, xw, yw

def main():

	rospy.init_node('coverage_path',anonymous=True)
	scanner = ScanPath(11, 32)
	# scanner.planning()
	sub_y = rospy.Subscriber('boundary', boundary, scanner.get_boundary)
	rospy.spin()

if __name__ == '__main__': 
	main()