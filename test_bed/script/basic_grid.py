import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt
from matplotlib.path import Path

EXTEND_AREA = 100.0

expand_x= [-1,1,0,0]
expand_y= [ 0,0,-1,1]
dx = [-1, 0, 1, 0]
dy = [ 0, 1, 0,-1]
right = 1
left  = -1

def move_target_grid(x_index, y_index, sweep_dir, moving_dir, occupancy_map_extended):

	n_x_index = x_index + moving_dir
	n_y_index = y_index
	#print("next x ", n_x_index, "next y ", n_y_index)
	#print(len(occupancy_map_extended),len(occupancy_map_extended[0]))
	# print("=======17 5======", occupancy_map_extended[5][17])
	if occupancy_map_extended[n_y_index][n_x_index] == True:
		return n_x_index, n_y_index, moving_dir
	else:
		n_x_index , n_y_index = find_safe_turning_grid(x_index, y_index, moving_dir, sweep_dir, occupancy_map_extended)
		if n_x_index is None and n_y_index is None:
			n_x_index = moving_dir +x_index
			n_y_index = y_index
		else:
			moving_dir *=-1
	return n_x_index, n_y_index, moving_dir


def find_safe_turning_grid(x_index, y_index, moving_dir, sweep_dir, occupancy_map_extended):
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


def cal_sweep_dir(start_y_indx, end_idx_y):
	if start_y_indx - end_idx_y > 0:
		print("-1 : move upper")
		sweep_dir = -1 #upper
	else:
		print("1 : move donwer")
		sweep_dir = 1 #downer
	return sweep_dir


def sweep_path_search(x_index, y_index, end_idx_x, end_idx_y, occupancy_map_extended):
	px, py = [], []
	sweep_dir = cal_sweep_dir(y_index, end_idx_y)
	moving_dir = 1
	while True:
		x_index, y_index, moving_dir = move_target_grid(x_index, y_index, sweep_dir, moving_dir, occupancy_map_extended)

		px.append(x_index)
		py.append(y_index)
		print("px :", px)
		print("py :", py)
		if end_idx_x == x_index and end_idx_y == y_index:
			print("done")
			break
	return px, py


def find_index_goal(occupancy_map_extended, xw, yw, from_upper=True):
	y_indexes = []
	x_indexes = []
	occupied_map =[]
	print("xw: ",xw,"yw",yw )
	if from_upper:
		x_range = range(xw)[::-1] #-1칸 간격으로 역순으로 처음부터 끝까지. 34, 33, 32, 31 ~ 이런식인 듯
		y_range = range(yw)[::-1] #-> [32,31,30,,,,,,0]
		# print("x_range", x_range, "y_range", y_range)
		# print("x_range type", type(x_range), "y_range type", type(y_range))
		# print("x_range list", list(x_range), "y_range list", list(y_range))
	else:
		x_range = range(xw)
		y_range = range(yw) #->[0,1,2,3,,,,,,,32]
		
	for ix in x_range:
		for iy in y_range:
			if occupancy_map_extended[iy][ix] == True: # Is (i,j) in the convex hull? #비어 있다면
				occupied_map.append([ix,iy])
				y_indexes.append(iy)
				x_indexes.append(ix)
				#print("ix", ix, "iy", iy)
				for i in range(4):
					t_ix = ix + dx[i]
					t_iy = iy + dy[i]
					#print("f ix", t_ix, "f iy", t_iy)
					if occupancy_map_extended[iy][ix] == False: #extend area 
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

	print("xindx:" , x_indexes, "y_index", y_indexes)
	return x_indexes, y_indexes, end_idx_y, end_idx_x, occupied_map

def calc_grid_map_config(ox, oy, xy_resolution): 
	""" Calculates the size, and the maximum distances according to the the measurement center """ 
	min_x = round(min(ox) - EXTEND_AREA / 2.0) 
	min_y = round(min(oy) - EXTEND_AREA / 2.0) 
	max_x = round(max(ox) + EXTEND_AREA / 2.0) 
	max_y = round(max(oy) + EXTEND_AREA / 2.0) 

	xw = int(round((max_x - min_x) / xy_resolution)) 
	yw = int(round((max_y - min_y) / xy_resolution)) 

	print("min_x :", min_x, "max_x :", max_x,"diff :", round((max_x - min_x)))
	print("min_y :", min_y, "max_y :", max_y,"diff :", round((max_y - min_y)))
	print("The grid map is ", xw, "x", yw, ".") 

	return min_x, min_y, max_x, max_y, xw, yw

def gen_grid_map(ox, oy, resolution):
	min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(ox, oy, resolution)

	occupancy_map = np.zeros((y_w, x_w)) / 2

	center_x = int( round(-min_x / resolution)) # center x coordinate of the grid map
	center_y = int( round(-min_y / resolution)) # center y coordinate of the grid map

	return occupancy_map, min_x, max_x, min_y, max_y, resolution,x_w, y_w

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
					if occupancy_map[j+expand_y[k]][i+expand_x[k]] != 1: #[y][x]
						convex_inside_x.append(i+expand_x[k])
						convex_inside_y.append(j+expand_y[k]) #extend area more. 
						occupancy_map[j+expand_y[k]][i+expand_x[k]] = 1
	return occupancy_map, convex_inside_x, convex_inside_y

def main():

	# ox = [728, 735, 748, 761, 774, 722, 791, 805, 819, 833, 848, 863, 878, 720, 874, 724, 881, 720, 882, 720, 880, 720, 878, 720, 880, 721, 882, 720, 879, 720, 879, 724, 882, 721, 880, 721, 881, 721, 724, 744, 757, 771, 784, 798, 808, 822, 837, 852, 866, 881, 723, 730, 723, 731, 724, 731]
	# oy = [-47, -47, -48, -48, -49, -38, -39, -40, -40, -40, -41, -41, -41, -29, -31, -19, -21, -9, -10, 0, 0, 9, 10, 19, 21, 29, 32, 39, 42, 48, 52, 58, 63, 67, 72, 76, 82, 86, 85, 85, 86, 87, 87, 88, 89, 89, 90, 91, 92, 93, 94, 94, 103, 103, 113, 113]
	#ox =[702, 710, 720, 732, 745, 757, 769, 701, 788, 803, 816, 699, 816, 698, 818, 698, 820, 697, 828, 697, 826, 696, 823, 696, 823, 697, 820, 704, 820, 704, 712, 729, 739, 756, 770, 781, 795, 809, 820]
	#oy =[-129, -129, -129, -130, -131, -132, -132, -119, -124, -125, -126, -108, -115, -98, -105, -89, -95, -79, -84, -69, -74, -59, -63, -49, -52, -39, -42, -29, -31, -19, -19, -20, -20, -20, -20, -20, -20, -20, -21]

	xy_resolution = 10
	occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, xw, yw = gen_grid_map(ox, oy, xy_resolution)
	# for i in range()

	xy_res = np.array(occupancy_map).shape #x y resolution
	print("xy_res", xy_res)
	grid_x = [(x - min_x)/xy_resolution for x in ox]
	grid_y = [(y - min_y)/xy_resolution for y in oy] #adjust 
	print("grid_x len :", len(grid_x))
	print("grid_x len :", len(grid_x))
	# sensor start position 
	sensor_center = [int(min(grid_x)), int(min(grid_y))]
	plt.plot(5, 5,'yo',label='occupancy_map')
	plt.plot(sensor_center[0], sensor_center[1], 'go', label='sensor')

	points = []
	for i in range(len(grid_x)):
		points.append([grid_x[i],grid_y[i]])
	points = np.array(points)
	hull = ConvexHull(points)
	hull_path = Path( points[hull.vertices] )

	for simplex in hull.simplices:
		plt.plot(points[simplex, 0], points[simplex, 1], 'r-')
	#plt.plot(points[simplex, 0], points[simplex, 1], 'r-', label='workpiece')
	occupancy_map_extended, convex_inside_x, convex_inside_y = covnex_insidecheck(occupancy_map, hull_path, occupancy_map, xw, yw)

	x_indexes, y_indexes, end_idx_y, end_idx_x, occupied_map = find_index_goal(occupancy_map_extended, xw, yw, from_upper=True)
	print("end idx y: ", end_idx_y,"end idx x: ", end_idx_x)
	start_x_indx = sensor_center[0]#min(x_indexes)
	start_y_indx = sensor_center[1]#min(y_indexes)
	px, py = sweep_path_search(start_x_indx, start_y_indx,end_idx_x, end_idx_y, occupancy_map_extended)
	px_real = [ num * 10 for num in px ]
	py_real = [ num * 10 for num in py ]
	path = zip(px_real, py_real)
	print("px :", px_real)
	print("py :", py_real)
	print("path :", path)

	plt.plot(px, py, '-k', label='scan path')
	sx = int(sensor_center[0])
	sy = int(sensor_center[1])
	plt.plot(convex_inside_x, convex_inside_y, 'bo')

	plt.imshow(occupancy_map, label='occupancy map')
	# scalebar = ScaleBar(0.1, 'cm', length_fraction='0.05', location='lower right') # 1 pixel = 0.2 meter
	# plt.gca().add_artist(scalebar)
	plt.gca().set_xticks(np.arange(0, xy_res[1], 1)) 
	plt.gca().set_yticks(np.arange(0, xy_res[0], 1)) 
	plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
	plt.grid(True, which="major", color="w", linewidth=0.6, alpha=0.5) # plot grid
	plt.legend(loc='best')
	title_font = {
    'fontsize': 16,
    'fontweight': 'bold'
	}
	label_font = {
    'fontsize': 12,
	}
	plt.title('scan path planning',fontdict=title_font)
	#plt.plot(x_indexes, y_indexes, '+r')
	plt.xlabel('X (1pixel=10mm)',fontdict=label_font)
	plt.ylabel('Y (1pixel=10mm)',fontdict=label_font)
	plt.subplot(111)
	plt.show()

if __name__ == '__main__': 
	main()
