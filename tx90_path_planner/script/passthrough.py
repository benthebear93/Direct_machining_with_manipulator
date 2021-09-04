import pcl
import numpy as np
import matplotlib.pyplot as plt
from random import randint

def visualization2D_xyz(new_cloud_data):
    # set the size of pyplot charts
    plt.rcParams['figure.figsize'] = (14, 6)

    # split the rgb column into three columns: red, green and blue
    rgb_columns = np.asarray(random_color_gen())

    # normalize the rgb values (they should be between [0, 1])
    rgb_columns = (rgb_columns / 255.0).astype(np.float)

    # plot the points of the columns
    plt.scatter(new_cloud_data[:, [0]], -new_cloud_data[:, [1]], color=rgb_columns)

    # scale the axis equally 
    # (Note: append a semicolon to your last line to prevent jupyter notebook 
    # from outputting your last cell's content)
    plt.axis('scaled');
    print("(x) : {:2.1f}m".format(new_cloud_data[:,0:1].max() - new_cloud_data[:,0:1].min()))
    print("(y) : {:2.1f}m".format(new_cloud_data[:,1:2].max() - new_cloud_data[:,1:2].min()))
    print("(z) : {:2.1f}m".format(new_cloud_data[:,2:3].max() - new_cloud_data[:,2:3].min()))
    plt.show()


def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):

    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def random_color_gen():

    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]

filepath = '/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/tx90_path_planner/pcd_data'
cloud = pcl.load(filepath + "/surface.pcd") # Deprecated; use pcl.load instead.
filter_axis = 'z'
axis_min = 0.24
axis_max = 0.3
cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)
pcl.save(cloud, filepath + "/surface2.pcd")