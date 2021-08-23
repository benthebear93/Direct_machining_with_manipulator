#!/usr/bin/env python2
""" 
subscriber for keyence LJ sensor profile (800 points)\

Author : Haegu Lee

"""
import rospy
from std_msgs.msg import Float32, Float64, Int32
from sensor_msgs.msg import PointCloud2
import math as m
import numpy as np
import time

class Laser_data:
    def __init__(self):
        self.pre_val = 0
        self.curr_val = 0
        self.num_data_count = 0
        self.avg_curr_val = 0
        self.temp =0
        rospy.init_node('profile_sub', anonymous=True)
        rospy.Subscriber("/profiles",PointCloud2,self.callback)

    def callback(self, msg):
        print(type(msg))

    def run(self):
        rospy.spin()

if __name__=='__main__':
    laser = Laser_data()
    laser.run()
