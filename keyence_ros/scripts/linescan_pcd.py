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
        rospy.Subscriber("arr_profile", Float64MultiArray, self.callback_pptk)

    def pptk_view(self, save_point):
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
        v = pptk.viewer(final_xyz)
        v.set(point_size=0.000001)

    def callback_pptk(self, msg):
        vector = msg.data
        self.save_point.append(vector)
        end = len(self.save_point)
        print("end:", end)
        if end>400:
            save_npy = np.array(self.save_point)
            np.save('/home/benlee/catkin_ws/src/Direct_machining_with_manipulator/keyence_ros/pcd_files/test_save2', save_npy) # x_save.npy
            rospy.sleep(1000000000000) # Sleeps for 1 sec
        #print("savep ", pptk.save_point[0][0]," ", pptk.save_point[0][1]," ", pptk.save_point[0][2])
        #print("save_point", len(pptk_c.save_point))
        # if end > 500 and pptk_c.flag ==0:
        #     pptk_c.flag =1

if __name__=='__main__':
    pptk_c = PPTK()
    rospy.init_node('linescan_save', anonymous=True)
    rospy.spin()