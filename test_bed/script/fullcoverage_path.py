#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray
from matplotlib.animation import FuncAnimation
from test_bed.msg import boundary
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
from gridbasesweepsearch import SweepSearcher
from matplotlib.path import Path
EXTEND_AREA = 50.0

class CoverPath():
    def __init__(self):
        self.t = 0
        self.x_data = [] # 
        self.y_data = []
        self.max_x = 0
        self.max_y = 0
        self.min_x = 0
        self.min_y = 0
        self.path = []
        # variables
        self.fig, self.ax = plt.subplots()
        # self.fig2, self.ax2 = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.sensor, = plt.plot([], [], 'b')
        self.sensor_center, = plt.plot([],[],'go')
        self.x_data, self.y_data = [] , []

        # figure variables
    def calc_grid_map_config(self, ox, oy, xy_resolution): 
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

    def get_boundary(self, data):
        self.t +=0.001
        #print "getting boundar x"
        self.x_data = data.boundary_x
        self.max_x = max(data.boundary_x)
        self.min_x = min(data.boundary_x)
        self.size_x = self.max_x -  self.min_x
        # print "size_x", self.size_x #0.11338

        self.y_data = data.boundary_y
        self.max_y = max(data.boundary_y)
        self.min_y = min(data.boundary_y)
        self.size_y = self.max_y -  self.min_y
        self.x_data = [ int(x * 1000) for x in self.x_data]
        self.y_data = [ int(y * 1000) for y in self.y_data]
        # print "size_y", self.size_y #0.12584

    def find_sweep_direction_and_start_position(self, ox, oy):
        # find sweep_direction
        max_dist = 0.0
        vec = [0.0, 0.0]
        sweep_start_pos = [0.0, 0.0]
        for i in range(len(ox) - 1):
            dx = ox[i + 1] - ox[i]
            dy = oy[i + 1] - oy[i]
            d = np.hypot(dx, dy)

            if d > max_dist:
                max_dist = d
                vec = [dx, dy]
                sweep_start_pos = [ox[i], oy[i]]

        return vec, sweep_start_pos

    def sensor_init(self):
        self.sensor.set_data([],[])
        return self.sensor

    def update_sensor(self, frame):
        start_x = (0.680 - self.min_x)*1000
        start_y = (-0.026- self.min_y)*1000
        self.sensor.set_data([start_x, start_x], [start_y, start_y+32]) #z = 0 -> x width = 32mm
        print(start_x, start_y)
        return self.sensor

    def sensor_center_init(self):
        self.sensor_center.set_data([],[])
        return self.sensor

    def update_sensor_center(self, frame):
        self.sensor_center.set_data(self.path[0][0], self.path[0][1]) #z = 0 -> x width = 32mm #+self.t

    def make_grid(self, xy_resolution):
        resolution = 1
        min_x, min_y, max_x, max_y, x_w, y_w = self.calc_grid_map_config(self.x_data, self.y_data, resolution)
        occupancy_map = np.zeros((x_w, y_w)) / 2
        center_x = int( round(-min_x / resolution)) # center x coordinate of the grid map
        center_y = int( round(-min_y / resolution)) # center y coordinate of the grid map

        # grid_x = [ int(x * 1000) for x in self.x_data]
        # grid_y = [ int(y * 1000) for y in self.y_data]
        return occupancy_map, min_x, max_x, min_y, max_y, resolution,x_w, y_w #grid_x, grid_y

    def grid_init(self):

        """
        make grid
        """
        xy_resolution = 1
        grid_x, grid_y = self.make_grid()
        sweep_start_position = [min(grid_x), max(grid_y)]
        print(sweep_start_position)

    def plot_init(self): #init

        self.ln.set_data([],[])

        # upper_x = self.max_x +50
        # lower_x = self.min_x -50
        # upper_y = self.max_y +50
        # lower_y = self.min_y -50
        # print ("upper_x: ",upper_x, "upper_y: ", upper_y)
        # print ("lower_x: ",lower_x, "lower_y", lower_y)
        # self.ax.set_xlim(lower_x, upper_x)
        # self.ax.set_ylim(lower_y, upper_y)
        # self.ax.set_xlim(-0.2, 0.05)
        # self.ax.set_ylim(0.0, 0.25)
        # self.ax.set_xlim(0.6, 0.9)
        # self.ax.set_ylim(-0.2, 0.1)
        # plt.grid()
        return self.ln

    def update_plot(self, frame): #animate
        print("update")
        """
        convex hull
        """
        #self.grid_init()
        xy_resolution = 1
        occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, xw, yw = self.make_grid(xy_resolution)
        #print("occupancy_map: ", occupancy_map)
        points = []
        xy_res = np.array(occupancy_map).shape
        plt.imshow(occupancy_map)
        plt.gca().set_xticks(np.arange(0, xy_res[1], 1), minor=True) 
        plt.gca().set_yticks(np.arange(0, xy_res[0], 1), minor=True) 
        plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        plt.grid(True, which="major", color="w", linewidth=0.6, alpha=0.5)
        grid_x = [x - min_x for x in self.x_data]
        grid_y = [y - min_y for y in self.y_data]

        points = []
        if len(self.x_data)>0:
            for i in range(len(grid_x)):
                points.append([grid_x[i],grid_y[i]])
            points = np.array(points)
            hull = ConvexHull(points)
            hull_path = Path(points[hull.vertices])
            # for i in range(len(self.x_data)):
            #     points.append([self.x_data[i],self.y_data[i]])
            # points = np.array(points)
            # hull = ConvexHull(points)

            # # hull = ConvexHull(a)

            # #plt.plot(points[:,0], points[:,1], 'ro')

            for simplex in hull.simplices:
                plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln

    def sensor_start(self):
        start_x = (0.680)
        start_y = ((-0.026+0.006)/2)
        self.path.append((start_x, start_y))

def main():
    # sweep_test = SweepSearcher(moving_direction=SweepSearcher.MovingDirection.RIGHT,
    #          sweeping_direction=SweepSearcher.SweepDirection.UP)
    rospy.init_node('coverage_path',anonymous=True)
    cover = CoverPath()
    cover.sensor_start()
    # sweep_test.planning_animation(cover.x_data, cover.y_data)
    sub_y = rospy.Subscriber('boundary', boundary, cover.get_boundary)
    print(len(cover.x_data))
    ani = FuncAnimation(cover.fig, cover.update_plot, frames=None, init_func=cover.plot_init)
    ani2 = FuncAnimation(cover.fig, cover.update_sensor, frames=None, init_func=cover.sensor_init)
    ani3 = FuncAnimation(cover.fig, cover.update_sensor_center, frames=None, init_func=cover.sensor_center_init)
    plt.show() 


if __name__ =="__main__":
    main()

