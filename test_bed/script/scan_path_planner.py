#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
from std_msgs.msg import Int32MultiArray
from test_bed.msg import boundary

import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.animation import FuncAnimation
EXTEND_AREA = 100.0
expand_x = [-1, 1, 0, 0]
expand_y = [0 ,0, -1, 1]
dx = [-1, 0, 1, 0]
dy = [ 0, 1, 0,-1]

class ScanPath():
	def __init__(self):
		self.x_index = []
		self.y_index = []
		self.expand_x = [-1, 1, 0, 0]
		self.expand_y = [0 ,0, -1, 1]
		self.x_data = []
		self.y_data = []
		self.fig, self.ax = plt.subplots()
		self.ln, = plt.plot([], [], 'ro')
		self.flag_pub = rospy.Publisher('process_flag', Int32, queue_size=10)

	def find_safe_turning_grid(self, x_index, y_index, moving_dir, sweep_dir, occupancy_map_extended):
		# print("find turning?")
		turing_window = [
				(moving_dir, 0), # right next
				(moving_dir, sweep_dir), #move and go up or down
				(0, sweep_dir),#only go up and down
				(-moving_dir, sweep_dir), #move opposite and go up or down
			]
		for (d_x_ind, d_y_ind) in turing_window:
			n_x_index = d_x_ind + x_index
			n_y_index = d_y_ind + y_index
			#print("turing next x ", d_x_ind, "next y ", d_y_ind)
			if occupancy_map_extended[n_y_index][n_x_index] == True:
				return n_x_index, n_y_index

		return None, None

	def move_target_grid(self, x_index, y_index, sweep_dir, moving_dir, occupancy_map_extended):

		n_x_index = x_index + moving_dir
		n_y_index = y_index
		print("next x ", n_x_index, "next y ", n_y_index)
		# print("=======17 5======", occupancy_map_extended[5][17])
		if occupancy_map_extended[n_x_index][n_y_index] == True:
			return n_x_index, n_y_index, moving_dir
		else:
			n_x_index , n_y_index = self.find_safe_turning_grid(x_index, y_index, moving_dir, sweep_dir, occupancy_map_extended)
			if n_x_index is None and n_y_index is None:
				n_x_index = moving_dir +x_index
				n_y_index = y_index
			else:
				moving_dir *=-1
		return n_x_index, n_y_index, moving_dir

	def cal_sweep_dir(self, start_y_indx, end_idx_y):
		if start_y_indx - end_idx_y > 0:
			print("-1 : move upper")
			sweep_dir = -1 #upper
		else:
			print("1 : move donwer")
			sweep_dir = 1 #downer
		return sweep_dir

	def sweep_path_search(self, x_index, y_index, end_idx_x, end_idx_y, occupancy_map_extended):
		px, py = [], []
		sweep_dir = self.cal_sweep_dir(y_index, end_idx_y)
		moving_dir = 1
		while True:
			x_index, y_index, moving_dir = self.move_target_grid(x_index, y_index, sweep_dir, moving_dir, occupancy_map_extended)

			px.append(x_index)
			py.append(y_index)

			if end_idx_x == x_index and end_idx_y == y_index:
				print("done")
				break
		return px, py

	def find_index_goal(self, occupancy_map_extended, xw, yw, from_upper=True):
		y_indexes = []
		x_indexes = []
		occupied_map =[]
		print("xw: ",xw,"yw",yw )

		if from_upper:
			x_range = range(xw)[::-1] 
			y_range = range(yw)[::-1] #-> [32,31,30,,,,,,0]
			# print("x_range", x_range, "y_range", y_range)
			# print("x_range type", type(x_range), "y_range type", type(y_range))
			# print("x_range list", list(x_range), "y_range list", list(y_range))
		else:
			x_range = range(xw)
			y_range = range(yw) #->[0,1,2,3,,,,,,,32]
		test_x = np.array(x_range)
		test_y = np.array(y_range)

		search_map =[]
		temp =[]
		for ix in x_range:
			for iy in y_range:
				if occupancy_map_extended[iy][ix] == True: #
					occupied_map.append([ix,iy])
					y_indexes.append(iy)
					x_indexes.append(ix)
					#print("ix", ix, "iy", iy)
					for i in range(4):
						t_ix = ix + dx[i]
						t_iy = iy + dy[i]
						#print("f ix", t_ix, "f iy", t_iy)
						if occupancy_map_extended[iy][iy] == False: #extend area 
							occupied_map.append([t_ix,t_iy])
							y_indexes.append(t_iy)
							x_indexes.append(t_ix)
		if from_upper:
			end_idx_y = max(y_indexes)
		else:
			end_idx_y = min(y_indexes)

		end_idx_x = []
		for i in range(len(occupied_map)):
			#print("ans : ", occupied_map[i][0], occupied_map[i][1]) #[0]= x,[1]=y
			if occupied_map[i][1] == end_idx_y:
				end_idx_x.append(occupied_map[i][0])

		if from_upper:
			end_idx_x = min(end_idx_x)
		else:
			end_idx_x = max(end_idx_x)

		# print("xindx:" , x_indexes, "y_index", y_indexes)
		return x_indexes, y_indexes, end_idx_y, end_idx_x, occupied_map


	def get_boundary(self, data):
		self.x_data = data.boundary_x
		self.y_data = data.boundary_y

		max_x = max(data.boundary_x)
		min_x = min(data.boundary_x)
		max_y = max(data.boundary_y)
		min_y = min(data.boundary_y)

		self.x_data = [ int(x * 1000) for x in self.x_data]
		self.y_data = [ int(y * 1000) for y in self.y_data]
		print("x_data", self.x_data)
		print("y_data", self.y_data)
		# occupancy_map, min_x, max_x, min_y, max_y, xy_resolution,xw, yw = gen_grid_map(self.x_data, self.y_data, 10)

		# xy_res = np.array(occupancy_map).shape #x y resolution
		# grid_x = [(x - min_x)/xy_resolution for x in self.x_data]
		# grid_y = [(y - min_y)/xy_resolution for y in self.y_data] #adjust 
		# sensor_center = [int(min(grid_x)), int(min(grid_y))]

		# points = []
		# for i in range(len(grid_x)):
		# 	points.append([grid_x[i],grid_y[i]])
		# points = np.array(points)
		# hull = ConvexHull(points)
		# hull_path = Path( points[hull.vertices] )
		# occupancy_map_extended, convex_inside_x, convex_inside_y = covnex_insidecheck(occupancy_map, hull_path, occupancy_map, xw, yw)

		# x_indexes, y_indexes, end_idx_y, end_idx_x, occupied_map = self.find_index_goal(occupancy_map_extended, xw, yw, from_upper=True)
		# start_x_indx = sensor_center[0]#min(x_indexes)
		# start_y_indx = sensor_center[1]#min(y_indexes)
		# px, py = self.sweep_path_search(start_x_indx, start_y_indx, end_idx_x, end_idx_y, occupancy_map_extended)
		# px_real = [ num * 10 for num in px ]
		# py_real = [ num * 10 for num in py ]
		# path = zip(px_real, py_real)
		# print("px :", px_real)
		# print("py :", py_real)
		# print("path :", path)

def calc_grid_map_config(ox, oy, xy_resolution): 
	""" Calculates the size, and the maximum distances according to the the measurement center """ 
	min_x = round(min(ox) - EXTEND_AREA / 2.0) 
	min_y = round(min(oy) - EXTEND_AREA / 2.0) 
	max_x = round(max(ox) + EXTEND_AREA / 2.0) 
	max_y = round(max(oy) + EXTEND_AREA / 2.0) 

	xw = int(round((max_x - min_x) / xy_resolution)) 
	yw = int(round((max_y - min_y) / xy_resolution)) 

	# print("min_x :", min_x, "max_x :", max_x,"diff :", round((max_x - min_x)))
	# print("min_y :", min_y, "max_y :", max_y,"diff :", round((max_y - min_y)))
	# print("The grid map is ", xw, "x", yw, ".") 

	return min_x, min_y, max_x, max_y, xw, yw

def gen_grid_map(ox, oy, resolution):
	min_x, min_y, max_x, max_y, xw, yw = calc_grid_map_config(ox, oy, resolution)

	occupancy_map = np.zeros((yw, xw)) / 2

	center_x = int( round(-min_x / resolution)) # center x coordinate of the grid map
	center_y = int( round(-min_y / resolution)) # center y coordinate of the grid map

	return occupancy_map, min_x, max_x, min_y, max_y, resolution, xw, yw

def covnex_insidecheck(occupancy_map, path, grid, xw, yw):
	convex_inside_x=[]
	convex_inside_y=[]
	for i in range(0,xw):
		for j in range(0,yw):
			if path.contains_point((i,j))==True: # Is (i,j) in the convex hull?
				convex_inside_x.append(i)
				convex_inside_y.append(j)
				occupancy_map[j][i] = 1 #[y][x]
				for k in range(0, 3):
					convex_inside_x.append(i+expand_x[k])
					convex_inside_y.append(j+expand_y[k]) #extend area more. 
					occupancy_map[j+expand_y[k]][i+expand_x[k]] = 1

	return occupancy_map, convex_inside_x, convex_inside_y

def main():
	rospy.init_node("scan_planner", anonymous=True)
	scanner = ScanPath()
	sub = rospy.Subscriber('boundary', boundary, scanner.get_boundary)
	rospy.spin()

if __name__ == '__main__': 
	main()

