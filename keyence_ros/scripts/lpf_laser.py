#!/usr/bin/env python2
""" 
lpf_laser node is low-pass-filter for Laser sensor(LJ-V7000, Keyence)

Author : Haegu Lee

"""
import math as m
import rospy
from std_msgs.msg import Float32, Float64, Int32
import numpy as np
import time

#end_time = 0 
#start_time = 0

class Laser_data:
    def __init__(self):
        self.pre_val = 0
        self.curr_val = 0
        self.num_data_count = 0
        self.avg_curr_val = 0
        self.temp =0

    def LowPassFilter(self, tau):
        self.curr_val = (tau*self.pre_val + self.curr_val*ts)/(tau + ts)

    def AverageFilter(self, n):
        if(self.num_data_count==n):
            #print(self.num_data_count)
            self.temp = self.temp/6
            self.avg_curr_val = self.temp
            self.num_data_count = 0
        self.temp = self.temp+self.curr_val
        self.num_data_count =self.num_data_count+1
            

def callback_kf(pose_data):
    #global end_time, start_time
    #start_time = time.time()
    get_z = pose_data.data
    laser.curr_val = get_z
    laser.LowPassFilter(0.01)
    pub.publish(laser.curr_val)
    laser.pre_val = laser.curr_val
    laser.AverageFilter(5)
    avg_val = laser.avg_curr_val*100
    pub2.publish(avg_val)
    #laser.temp = laser.avg_curr_val


    #end_time = time.time()
    #print("ts : ", end_time - start_time)

def get_measurement():
    print("start get m")
    rospy.Subscriber("mid_laser",Float32,callback_kf)
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('lpf_laser', anonymous=True)
    pub = rospy.Publisher('lpf_z',Float32, queue_size =10)
    pub2 = rospy.Publisher('avg_z',Int32, queue_size =10)
    print("dt :",ts)
    laser = Laser_data()
    get_measurement()
