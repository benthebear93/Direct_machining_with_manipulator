import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
import math

xdata =[702, 710, 720, 732, 745, 757, 769, 701, 788, 803, 816, 699, 816, 698, 818, 698, 820, 697, 828, 697, 826, 696, 823, 696, 823, 697, 820, 704, 820, 704, 712, 729, 739, 756, 770, 781, 795, 809, 820]
ydata =[-129, -129, -129, -130, -131, -132, -132, -119, -124, -125, -126, -108, -115, -98, -105, -89, -95, -79, -84, -69, -74, -59, -63, -49, -52, -39, -42, -29, -31, -19, -19, -20, -20, -20, -20, -20, -20, -20, -21]
offset_grid =10
resolution = 1
width = math.ceil((max(xdata) - min(xdata)) / resolution) + offset_grid
height = math.ceil((max(ydata) - min(ydata)) / resolution) + offset_grid
center_x = (max(xdata) + min(xdata)) / 2.0
center_y = (max(ydata) + min(ydata)) / 2.0
left_lower_x = center_x - width / 2.0 * resolution
left_lower_y = center_y - height / 2.0 * resolution
ndata = width * height
data = [0] * ndata
print("c_x:",center_x, "c_y", center_y, "width", width,"height", height )
print("left_lower_x:",left_lower_x, "left_lower_y", left_lower_y)
print("ndata:",ndata)

def calc_grid_central_xy_position_from_index(index, lower_pos):
    return lower_pos + index * 1 + 1 / 2.0

def calc_grid_central_xy_position_from_xy_index(x_ind, y_ind):
    x_pos = self.calc_grid_central_xy_position_from_index(x_ind, left_lower_x)
    y_pos = self.calc_grid_central_xy_position_from_index(y_ind, left_lower_y)

def check_inside_polygon(iox, ioy, x, y):

    npoint = len(x) - 1
    inside = False
    for i1 in range(npoint):
        i2 = (i1 + 1) % (npoint + 1)

        if x[i1] >= x[i2]:
            min_x, max_x = x[i2], x[i1]
        else:
            min_x, max_x = x[i1], x[i2]
        if not min_x < iox < max_x:
            continue

        tmp1 = (y[i2] - y[i1]) / (x[i2] - x[i1])
        if (y[i1] + tmp1 * (iox - x[i1]) - ioy) > 0.0:
            inside = not inside
    return inside

def set_value_from_xy_index(x_ind, y_ind, val):
    global width, data, ndata
    """set_value_from_xy_index

    return bool flag, which means setting value is succeeded or not

    :param x_ind: x index
    :param y_ind: y index
    :param val: grid value
    """

    if (x_ind is None) or (y_ind is None):
        return False, False

    grid_ind = int(y_ind * width + x_ind)

    if 0 <= grid_ind < ndata:
        data[grid_ind] = val
        return True  # OK
    else:
        return False  # NG

def plot_grid_map( ax=None):
    global width, data, ndata
    grid_data = np.reshape(np.array(data), (height, width))
    if not ax:
        fig, ax = plt.subplots()
    heat_map = ax.pcolor(grid_data, cmap="Blues", vmin=0.0, vmax=1.0)
    plt.axis("equal")
    plt.show()

    return heat_map

if (xdata[0] != xdata[-1]) or (ydata[0] != ydata[-1]):
    xdata.append(xdata[0])
    ydata.append(ydata[0])
inside=True

for x_ind in range(width):
    for y_ind in range(height):
        x_pos = calc_grid_central_xy_position_from_index(x_ind, left_lower_x)
        y_pos = calc_grid_central_xy_position_from_index(y_ind, left_lower_y)
        flag = check_inside_polygon(x_pos, y_pos, xdata, ydata)
        if flag is inside:
            set_value_from_xy_index(x_ind, y_ind, 1.0)
plot_grid_map()


# points = []
# for i in range(len(xdata)):
#     points.append([xdata[i],ydata[i]])

# points = np.array(points)

# hull = ConvexHull(points)

# # hull = ConvexHull(a)

# plt.plot(points[:,0], points[:,1], 'o')

# for simplex in hull.simplices:

#     plt.plot(points[simplex, 0], points[simplex, 1], 'k-')

# plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'r--', lw=2)
# plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'ro')
# plt.plot(center_x, center_y, 'bo')
# plt.show()