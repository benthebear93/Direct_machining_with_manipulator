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
from matplotlib.path import Path
from std_msgs.msg import Int32
import time

class ScanPath:
	def __init__(self, xy_resolution, sensor_x_length):
		self.xy_resolution = xy_resolution  # 11mm for one grid
		self.sensor_x_length = sensor_x_length  # 32mm
		self.path_pub = rospy.Publisher("path", scan_path, queue_size=10)
		self.points = 0
		self.expand_x = [-1, 1, 0, 0]
		self.expand_y = [0, 0, -1, 1]	
		self.EXTEND_AREA = 400.0
		
		self.hull = None
		self.hull_path = None

	def planning(
		self,
		x_start,
		y_start,
		x_end,
		y_end,
		x_end2,
		y_end2,
		sweep_dir,
		linechange_size,
		grid_map,
	):

		line_counter = 0
		x_pos = x_start
		y_pos = y_start  # scanning start position

		px = []
		py = []

		flag = 0
		px.append(x_pos)
		py.append(y_pos)
		while True:
			if x_pos < x_end:

				n_x_pos = x_pos
				n_y_pos = y_pos + sweep_dir  # scan in y direction

				# inside of the occupancy map
				if grid_map[n_y_pos][n_x_pos] == True:
					px.append(n_x_pos)
					py.append(n_y_pos)

					x_pos = n_x_pos
					y_pos = n_y_pos
				# outside of the occupancy map
				else:
					x_pos, y_pos, sweep_dir = change_sweep_dir(
						x_pos, y_pos, sweep_dir, linechange_size
					)
					if grid_map[y_pos+sweep_dir][x_pos] == True:
						px.append(x_pos)
						py.append(y_pos)
					line_counter += 1
			# last scan line
			else:
				n_y_pos = y_pos + sweep_dir
				n_x_pos = x_pos
				if grid_map[n_y_pos][n_x_pos] == True:
					px.append(n_x_pos)
					py.append(n_y_pos)
					x_pos = n_x_pos
					y_pos = n_y_pos
				else:
					break
				# if sweep_dir > 0:
				# 	if n_y_pos > y_end:
				# 		break
				# else:
				# 	if n_y_pos < y_start:
				# 		break
				# x_pos = n_x_pos
				# y_pos = n_y_pos
				# px.append(x_pos)
				# py.append(y_pos)

		xy_res = np.array(grid_map).shape  # x y resolution
		plt.figure()
		im = plt.imshow(grid_map, label="occupancy map")
		ax = plt.gca()

		for simplex in self.hull.simplices:
			ax.plot(self.points[simplex, 0], self.points[simplex, 1], 'r-', label='workpiece boundary')

		ax.set_title("Grid based Coverage Path planning", fontsize=17)
		ax.set_xlabel('y (index)', fontsize = 16)
		ax.set_ylabel('x (index)', fontsize = 16)

		ax.plot(px, py, "-k", label="scan path")
		ax.scatter(px,py, label="way points")

		ax.xaxis.set_ticks_position('both')
		ax.yaxis.set_ticks_position('both')

		ax.set_xticks(np.arange(0, xy_res[1], 2))
		ax.set_yticks(np.arange(0, xy_res[0], 2))

		ax.tick_params(axis="x", direction="in", which='major', color="w", labelsize=13)
		ax.tick_params(axis="x", direction="in", which='minor', color="w", labelsize=13)

		ax.tick_params(axis="y", direction="in", which='major',color="w", labelsize=13)
		ax.tick_params(axis="y", direction="in", which='minor',color="w", labelsize=13)

		ax.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
		ax.grid(True, which="major", color="w", linewidth=0.6, alpha=0.5)  # plot grid
		# ax.grid(color="#A2A6AB")
		ax.set_axisbelow(True)
		plt.legend(loc="best")
		plt.show()
		print("planning start")
		return px, py

	def get_boundary(self, data):

		# print("boundary data", data.boundary_x, data.boundary_y)
		x_data = data.boundary_x
		y_data = data.boundary_y
		x_data = [int(x*1000) for x in x_data]
		y_data = [int(y*1000) for y in y_data]  # change float to int

		#x_data =[702, 710, 720, 732, 745, 757, 769, 701, 788, 803, 816, 699, 816, 698, 818, 698, 820, 697, 828, 697, 826, 696, 823, 696, 823, 697, 820, 704, 820, 704, 712, 729, 739, 756, 770, 781, 795, 809, 820]
		#y_data =[-129, -123, -119, -113, -112, -110, -105, -119, -100, -100, -99, -97, -97, -105, -95, -89, -95, -79, -84, -69, -74, -59, -63, -49, -52, -39, -42, -29, -31, -19, -19, -20, -20, -20, -20, -20, -20, -20, -21]
		# x_data =[ 702, 710, 720, 732, 745, 757, 769, 701, 788, 803, 816, 699, 816, 698,  818, 698, 820, 697, 828, 697, 826, 696, 823, 696, 823, 697, 820, 704, 820, 704, 712, 729, 739, 756, 770, 781, 795, 809, 820]
		# y_data =[-129, -123, -119, -113, -112, -110, -105, -119, -100, -100, -99, -97, -97, -105, -98, -105, -89, -95, -79, -84, -69, -74, -60, -63, -60, -40, -60, -25, -60, -50, -60, -60, -60, -60, -50, -40, -30, -20, -10, -5]

		
		grid_resolution = int(round(self.sensor_x_length / self.xy_resolution))
		grid_map, min_x, max_x, min_y, max_y, self.xy_resolution = self.gen_grid_map(
			x_data, y_data, self.xy_resolution
		)
		grid_x = [(x - min_x) / self.xy_resolution for x in x_data]
		grid_y = [(y - min_y) / self.xy_resolution for y in y_data]  # expand scanning area in case
		print(
			"scan grid map is x: ",
			round(max(grid_x) - min(grid_x)),
			" y: ",
			round(max(grid_y) - min(grid_y)),
		)
		print(
			"scan grid map is x: ",
			(max_x - min_x) / self.xy_resolution,
			" y: ",
			(max_y - min_y) / self.xy_resolution,
		)

		x_w = int(round((max_x - min_x) / self.xy_resolution)) 
		y_w = int(round((max_y - min_y) / self.xy_resolution)) 

		self.points = []
		for i in range(len(grid_x)):
			self.points.append([grid_x[i],grid_y[i]])

		self.points = np.array(self.points)
		self.hull = ConvexHull(self.points)
		self.hull_path = Path( self.points[self.hull.vertices] )

		occupancy_map_extended, convex_index, convex_inside_x, convex_inside_y = self.covnex_insidecheck(grid_map, self.hull_path, x_w, y_w)

		corners = select_scan_area(convex_inside_x, convex_inside_y)
		# if start_pos = "right_bottom" #left top to
		x_start = int(round(corners[3][0]))
		y_start = int(round(corners[3][1]))
		x_end = int(round(corners[0][0]))
		y_end = int(round(corners[0][1]))
		x_end2 = int(round(corners[1][0]))
		y_end2 = int(round(corners[1][1]))

		if x_start % 2 != 0:
			if x_end % 2 == 0:
				x_end += 1
			if x_end2 % 2 == 0:
				x_end2 += 1
		else:
			if x_end % 2 != 0:
				x_end += 1
			if x_end2 % 2 != 0:
				x_end2 += 1
		# for i in range(y_start, y_end + 1):
		# 	for j in range(x_start, x_end + 1):
		# 		grid_map[i][j] = 1

		sweep_dir, linechange_size = sweep_dircheck(
			x_start, x_end, y_start, y_end, grid_resolution
		)
		px, py = self.planning(
			x_start,
			y_start,
			x_end,
			y_end,
			x_end2,
			y_end2,
			sweep_dir,
			linechange_size,
			occupancy_map_extended,
		)
		px = [(x * self.xy_resolution + min_x) / 1000 for x in px]
		py = [(y * self.xy_resolution + min_y) / 1000 for y in py]
		path = scan_path()
		path.path_x = px
		path.path_y = py
		# print(len(px), px)
		# print(len(py), py)
		self.path_pub.publish(path)

	def covnex_insidecheck(self, grid_map, path, xw, yw):
		convex_inside_x=[]
		convex_inside_y=[]
		convex_index = []
		for i in range(0,xw):
			for j in range(0,yw):
				if path.contains_point((i,j)) == True: # Is (i,j) in the convex hull?
					convex_inside_x.append(i)
					convex_inside_y.append(j)
					grid_map[j][i] = 1 #[y][x]
					convex_index.append([i, j])
					for k in range(0, 3):
						if grid_map[j+self.expand_y[k]][i+self.expand_x[k]] != 1: #[y][x]
							convex_inside_x.append(i+self.expand_x[k])
							convex_inside_y.append(j+self.expand_y[k]) #extend area more. 
							grid_map[j+self.expand_y[k]][i+self.expand_x[k]] = 1
							convex_index.append([j+self.expand_y[k], i+self.expand_x[k]])

		occupancy_map = grid_map

		return occupancy_map, convex_index , convex_inside_x, convex_inside_y
	def gen_grid_map(self, ox, oy, xy_resolution):

		min_x = round(min(ox) - self.EXTEND_AREA / 2.0)
		min_y = round(min(oy) - self.EXTEND_AREA / 2.0)
		max_x = round(max(ox) + self.EXTEND_AREA / 2.0)
		max_y = round(max(oy) + self.EXTEND_AREA / 2.0)

		xw = int(round((max_x - min_x) / xy_resolution)) # 573/11
		yw = int(round((max_y - min_y) / xy_resolution))

		print("min_x :", min_x, "max_x :", max_x, "diff :", round((max_x - min_x)))
		print("min_y :", min_y, "max_y :", max_y, "diff :", round((max_y - min_y)))
		print("max grid map is x: ", xw, " y: ", yw)
		grid_map = np.zeros((yw, xw))
		# print(grid_map.size)
		# grid_map = np.zeros((yw, xw)) / 2
		# print(grid_map.size)
		return grid_map, min_x, max_x, min_y, max_y, xy_resolution

def change_sweep_dir(x_pos, y_pos, sweep_dir, up_down):
	# print("changing line")

	sweep_dir *= -1
	x_pos = x_pos + up_down
	y_pos = y_pos

	return x_pos, y_pos, sweep_dir


def sweep_dircheck(x_start, x_end, y_start, y_end, linechange_size):
	if x_start < x_end:
		from_upper = False
		linechange_size = linechange_size  # should it go right? left?
	else:
		from_upper = True
		linechange_size = -linechange_size
	if y_start < y_end:
		sweep_dir = 1
	else:
		sweep_dir = -1
	return sweep_dir, linechange_size


def select_scan_area(grid_x, grid_y):

	left_top = [max(grid_x), max(grid_y)]
	right_top = [max(grid_x), min(grid_y)]
	left_bottom = [min(grid_x), max(grid_y)]
	right_bottom = [min(grid_x), min(grid_y)]
	corners = [left_top, right_top, left_bottom, right_bottom]
	
	return corners

def main():

	rospy.init_node("coverage_path", anonymous=True)
	scanner = ScanPath(11, 32)
	# scanner.planning()
	sub_y = rospy.Subscriber("boundary", boundary, scanner.get_boundary)
	rospy.spin()


if __name__ == "__main__":
	main()
