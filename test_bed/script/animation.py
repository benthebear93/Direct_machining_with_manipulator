#!/usr/bin/env python2
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Int32MultiArray

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 300)
        self.ax.set_ylim(0, 300)
        return self.ln

    def callback_x(self, msg):
        self.x_data = msg.data
        # x_index = len(self.x_data)
        # print("x:",self.x_data)
        # print("y:",self.y_data)
        # self.x_data.append(x_index+1)

    def callback_y(self, msg):
        self.y_data = msg.data
        # yaw_angle = msg.data
        # self.y_data.append(yaw_angle)
        # x_index = len(self.x_data)
        # print("x:",self.x_data)
        # print("y:",self.y_data)
        # self.x_data.append(x_index+1)
    
    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        return self.ln



rospy.init_node('msg_recive')
vis = Visualiser()
sub = rospy.Subscriber('boundary_x', Int32MultiArray, vis.callback_x)
sub2 = rospy.Subscriber('boundary_y', Int32MultiArray, vis.callback_y)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 