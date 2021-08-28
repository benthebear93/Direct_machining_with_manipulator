#!/usr/bin/env python2
""" 
ppkt visualize node
Author : Haegu Lee
"""
import math as m
import rospy
from std_msgs.msg import Float32, Float64, Int32, Float32MultiArray
import numpy as np
import time
import pptk

#end_time = 0 
#start_time = 0

class PPTK:
    def __init__(self):
        self.point_size = 0
        self.save_point =[]
        self.temp = 0
        self.final_pptk = []

def pptk_view(save_point):
    print(type(save_point[0][0]))
    print("pptk_view start")
    final_xyz =[]
    for a in range(0, len(save_point)):
        for i in range(0, len(save_point[a]),3):
            xyz =[]
            xyz.append(round(save_point[a][i], 4))
            xyz.append(round(save_point[a][i+1], 4))
            xyz.append(round(save_point[a][i+2], 4))
            final_xyz.append(xyz)
    final_xyz = np.array(final_xyz, dtype=np.float32)
    pptk_c.final_pptk = final_xyz

def callback_pptk(msg):
    vector = msg.data
    pptk_c.save_point.append(vector)

def get_measurement():
    print("start get vector")
    rospy.Subscriber("profile_sum", Float32MultiArray, callback_pptk)
    #print("end")
    rospy.spin()


if __name__=='__main__':
    rospy.init_node('pptk_view', anonymous=True)
    pptk_c = PPTK()
    get_measurement()
    if rospy.is_shutdown():
        print('save')
        pptk_view(pptk_c.save_point)
        np.save('/home/benlee/catkin_ws/src/pptk_save_reduce2', pptk_c.final_pptk)

    