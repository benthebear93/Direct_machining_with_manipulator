#!/usr/bin/env python2
""" 
ppkt visualize node

Author : Haegu Lee

"""
import math as m
import rospy
from std_msgs.msg import Float32, Float64, Int32, Float64MultiArray
import numpy as np
import time
import pptk

#end_time = 0 
#start_time = 0

class PPTK:
    def __init__(self):
        self.point_size = 0
        self.save_point =[]
        self.flag = 0
        self.temp = 0
        self.final_pptk = []


def callback_switch(msg):
    switch_flag = msg.data
    if(switch_flag == 1):
        pptk_view(pptk_c.save_point)
        np.save('/home/benlee/catkin_ws/src/pptk_save', pptk_c.final_pptk) # x_save.npy
    else:
        print("stop")

def pptk_view(save_point):
    print("pptk_view start")
    final_xyz =[]
    for a in range(0, len(save_point)):
        for i in range(0, len(save_point[a]),3):
            xyz =[]
            xyz.append(round(save_point[a][i], 4))
            xyz.append(round(save_point[a][i+1], 4))
            xyz.append(round(save_point[a][i+2], 4))
            final_xyz.append(xyz)
    final_xyz = np.array(final_xyz)
    pptk_c.final_pptk = final_xyz
    #print(final_xyz[0])
    #print(final_xyz[1])
    #print(final_xyz[2])
    #v = pptk.viewer(final_xyz)
    #v.set(point_size=0.000001)
    
    
def end_msg():
    print("viewer start")
            

def callback_pptk(msg):
    vector = msg.data
    pptk_c.save_point.append(vector)
    end = len(pptk_c.save_point)
    #print("savep ", pptk.save_point[0][0]," ", pptk.save_point[0][1]," ", pptk.save_point[0][2])
    #print("save_point", len(pptk_c.save_point))
    if end > 500 and pptk_c.flag ==0:
        pptk_c.flag =1


def get_measurement():
    print("start get vector")
    rospy.Subscriber("profile_sum", Float64MultiArray, callback_pptk)
    rospy.Subscriber("switch_flag", Int32, callback_switch)
    #print("end")
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('pptk_view', anonymous=True)
    pptk_c = PPTK()
    get_measurement()
