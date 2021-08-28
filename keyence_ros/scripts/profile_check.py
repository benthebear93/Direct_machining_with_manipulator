#!/usr/bin/env python2
""" 
profile_check node is for calibrating Laser sensor(LJ-V7000, Keyence)

Author : Haegu Lee

"""
import math as m
import rospy
from std_msgs.msg import Float32, Float64, Int32
import numpy as np
import time
from MoMach_ros.msg import pos_stream

class CONST:
    def __init__(self):
        self.ONE_ROTATION_DIS = 314.159
        self.FEED = 0.833 # 50mm/min 
        self.ONE_ROTATION_TIME = self.ONE_ROTATION_DIS/self.FEED
        self.end_time = 0
        self.start_time = 0
        self.init_flag = 0


class Laser_data:
    def __init__(self):
        self.init_z = 0
        self.pre_val = 0
        self.curr_val = 0
        self.gradient = 0
        self.count = 0

    def Gradient_cal(self, dt):
        #print("pre value :", self.pre_val, "current value :", self.curr_val, "time :", dt)
        self.gradient = (self.pre_val + self.curr_val)/(dt)
        #print("gradient value :", self.gradient)


def callback(pose_data):

    ''' kinematic error data generate by time'''

    global end_time, start_time,count
    laser.curr_val = pose_data.data
    laser.count = laser.count+1
    const.end_time = time.time()
    #print("laser count ", laser.count)
    w = -2*m.pi/const.ONE_ROTATION_TIME
    dt = const.end_time -const.start_time
    error_com = -0.2*m.sin(w*dt)-0.1*m.sin(9*w*dt)
    end_time=time.time()
    pub.publish(error_com)

def sub_pos_callback(y_data):

    ''' kinematic error data generate by y_data'''

    w = -2*m.pi/const.ONE_ROTATION_TIME
    t = (y_data.data-1.787)/const.FEED
    error_com = laser.init_z-0.15*m.sin(w*t)-0.08*m.sin(9*w*t)
    #print("difference = ", error_com-laser.curr_val)
    pub2.publish(error_com)

def init_pose(mid_laser):

    ''' get initial position of z aix
    make it as IVP '''
    if(const.init_flag == 0):
        laser.init_z = mid_laser.data
        const.init_flag = 1


def get_measurement():
    rospy.Subscriber("mid_laser",Float32,callback)
    rospy.Subscriber("posy", Float32, sub_pos_callback)
    rospy.Subscriber("mid_laser", Float32, init_pose)
    end_time=time.time()
    rospy.spin()

if __name__=='__main__':
    start_time = time.time()
    rospy.init_node('profile_check', anonymous=True)
    pub = rospy.Publisher('error_com1',Float32, queue_size =10)
    pub2 = rospy.Publisher('error_com2',Float32, queue_size =10)
    laser = Laser_data()
    const = CONST()
    const.start_time= start_time
    print("gradient :", laser.gradient)
    get_measurement()
    