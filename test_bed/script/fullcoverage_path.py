#!/usr/bin/env python2
import numpy as np

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray
from matplotlib.animation import FuncAnimation

class CoverPath():
    def __init__(self):
        self.x_data = [] # 
        self.y_data = []
        self.max_x = 0
        self.max_y = 0
        self.min_x = 0
        self.min_y = 0
        self.path = []
        # variables
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.sensor, = plt.plot([], [], 'b')
        self.x_data, self.y_data = [] , []
        # figure variables

    def get_x_boundary(self, data_x):
        self.x_data = data_x.data
        self.max_x = max(data_x.data)
        self.min_x = min(data_x.data)
        self.size_x = self.max_x -  self.min_x
        #print "size_x", self.size_x

    def get_y_boundary(self, data_y):
        self.y_data = data_y.data
        self.max_y = max(data_y.data)
        self.min_y = min(data_y.data)
        self.size_y = self.max_y -  self.min_y
        #print "size_y", self.size_y

    def plot_init(self):
        upper_x = self.max_x +50
        lower_x = self.min_x -50
        upper_y = self.max_y +50
        lower_y = self.min_y -50

        self.ax.set_xlim(lower_x, upper_x)
        self.ax.set_ylim(lower_y, upper_y)
        return self.ln

    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        self.sensor.set_data(self.min_x, self.min_y)
        return self.ln, self.sensor

    def sensor_start(self):
        start_x = self.min_x
        start_y = self.min_y
        self.path.append(start_x)
        self.path.append(start_y)



rospy.init_node('coverage_path',anonymous=True)
cover = CoverPath()
sub_X = rospy.Subscriber('boundary_x', Int32MultiArray, cover.get_x_boundary)
sub_y = rospy.Subscriber('boundary_y', Int32MultiArray, cover.get_y_boundary)

ani = FuncAnimation(cover.fig, cover.update_plot, init_func=cover.plot_init)
plt.grid()
plt.axis("equal")
plt.show(block=True) 
