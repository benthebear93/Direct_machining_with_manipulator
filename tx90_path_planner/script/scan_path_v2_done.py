import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib.path import Path
EXTEND_AREA = 100.0

def move_sensor(x_start, y_start, x_end, y_end, yw, sweep_dir, up_down, grid_map):
	print("planning start")
	line_counter = 1
	x_pos = x_start
	y_pos = y_start
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
	return px, py

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
	ox =[702, 710, 720, 732, 745, 757, 769, 701, 788, 803, 816, 699, 816, 698, 818, 698, 820, 697, 828, 697, 826, 696, 823, 696, 823, 697, 820, 704, 820, 704, 712, 729, 739, 756, 770, 781, 795, 809, 820]
	oy =[-129, -129, -129, -130, -131, -132, -132, -119, -124, -125, -126, -108, -115, -98, -105, -89, -95, -79, -84, -69, -74, -59, -63, -49, -52, -39, -42, -29, -31, -19, -19, -20, -20, -20, -20, -20, -20, -20, -21]

	# ox = [728, 735, 748, 761, 774, 722, 791, 805, 819, 833, 848, 863, 878, 720, 874, 724, 881, 720, 882, 720, 880, 720, 878, 720, 880, 721, 882, 720, 879, 720, 879, 724, 882, 721, 880, 721, 881, 721, 724, 744, 757, 771, 784, 798, 808, 822, 837, 852, 866, 881, 723, 730, 723, 731, 724, 731]
	# oy = [-47, -47, -48, -48, -49, -38, -39, -40, -40, -40, -41, -41, -41, -29, -31, -19, -21, -9, -10, 0, 0, 9, 10, 19, 21, 29, 32, 39, 42, 48, 52, 58, 63, 67, 72, 76, 82, 86, 85, 85, 86, 87, 87, 88, 89, 89, 90, 91, 92, 93, 94, 94, 103, 103, 113, 113]
	
	xy_resolution = 11 #11mm for one grid
	sensor_x_length = 32
	up_down_size = round(sensor_x_length/xy_resolution)
	print("up_down_size", up_down_size)
	grid_map, min_x, max_x, min_y, max_y, xy_resolution, xw, yw = gen_grid_map(ox, oy, xy_resolution)
	xy_res = np.array(grid_map).shape #x y resolution
	grid_x = [(x - min_x)/xy_resolution for x in ox]
	grid_y = [(y - min_y)/xy_resolution for y in oy] #adjust 
	# print("grid_x: ", grid_x, "len :", len(grid_x))
	# print("grid_y: ", grid_y, "len :", len(grid_y))
	plt.plot(grid_x, grid_y, 'go')
	corners = select_scan_area(grid_x, grid_y)

	for i in range(4):
		plt.plot(corners[i][0], corners[i][1], 'ko')
	
	plt.plot(corners[1][0], corners[1][1], '+r', label='sensor')
	x_start = round(corners[1][0])
	y_start = round(corners[1][1])
	x_end = round(corners[2][0])
	y_end = round(corners[2][1])
	print(x_start, y_start, x_end, y_end)
	for i in range(y_start,y_end+1):
		for j in range(x_start, x_end+1):
			grid_map[i][j] = 1

	from_upper, sweep_dir, up_down = sweep_updown(x_start, x_end, y_start, y_end, up_down_size)
	px , py = move_sensor(x_start, y_start, x_end, y_end,yw, sweep_dir, up_down, grid_map)
	plt.plot(px, py, '-ro')
	# points = []
	# for i in range(len(grid_x)):
	# 	points.append([grid_x[i],grid_y[i]])
	# points = np.array(points)
	# hull = ConvexHull(points)
	# hull_path = Path( points[hull.vertices] )
	# for simplex in hull.simplices:
	# 	plt.plot(points[simplex, 0], points[simplex, 1], 'ro')

	#x_indexes, y_indexes, end_idx_y, grid_map = find_index_and_extend(grid_map, xw, yw, from_upper=True)

	#print("end_idx_y",end_idx_y)
	plt.imshow(grid_map)
	plt.gca().set_xticks(np.arange(0, xy_res[1], 1)) 
	plt.gca().set_yticks(np.arange(0, xy_res[0], 1)) 
	plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
	plt.grid(True, which="major", color="w", linewidth=0.6, alpha=0.5) # plot grid
	plt.show()

if __name__ == '__main__': 
	main()