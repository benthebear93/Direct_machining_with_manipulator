#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import geometry_msgs
from math import pi
from std_msgs.msg import String
import numpy as np
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt
# def callback(data):
#     print("pos", data.points[0].positions)
    
def listener():
    rospy.init_node('trajplot', anonymous=True)
    msg = rospy.wait_for_message("/traj", JointTrajectory, timeout=None)
    print("point", msg.points[0].positions)
    j1 = []
    j2 = []
    j3 = []
    j4 = []
    j5 = []
    j6 = []
    j1_v = []
    j2_v = []
    j3_v = []
    j4_v = []
    j5_v = []
    j6_v = []
    time = []
    tempj1 = 0
    tempj2 = 0
    tempj3 = 0
    tempj4 = 0
    tempj5 = 0
    tempj6 = 0
    for i in range(0,len(msg.points)):
        j1.append(msg.points[i].positions[0])
        j1_v.append((tempj1-msg.points[i].positions[0])/0.5)
        tempj1 = msg.points[i].positions[0]

        j2.append(msg.points[i].positions[1])
        j2_v.append((tempj2-msg.points[i].positions[1])/0.5)
        tempj2 = msg.points[i].positions[1]

        j3.append(msg.points[i].positions[2])
        j3_v.append((tempj3-msg.points[i].positions[2])/0.5)
        tempj3 = msg.points[i].positions[2]

        j4.append(msg.points[i].positions[3])
        j4_v.append((tempj4-msg.points[i].positions[3])/0.5)
        tempj4 = msg.points[i].positions[3]

        j5.append(msg.points[i].positions[4])
        j5_v.append((tempj5-msg.points[i].positions[4])/0.5)
        tempj5 = msg.points[i].positions[4]

        j6.append(msg.points[i].positions[5])
        j6_v.append((tempj6-msg.points[i].positions[5])/0.5)
        tempj6 = msg.points[i].positions[5]


        time.append(0.5*i)
    plt.figure(1)
    plt.plot(time, j1, label='join1/positions')
    # plt.gca().set_title('J1')
    # plt.xlabel('time(s)')
    # plt.ylabel('joint radian')
    # plt.grid()

    # plt.subplot(2,3,2)
    plt.plot(time, j2, label='join2/positions')
    # plt.gca().set_title('J2')
    # plt.xlabel('time(s)')
    # plt.ylabel('joint radian')
    # plt.grid()

    # plt.subplot(2,3,3)
    plt.plot(time, j3, label='join3/positions')
    # plt.gca().set_title('J3')
    # plt.xlabel('time(s)')
    # plt.ylabel('joint radian')
    # plt.grid()

    # plt.subplot(2,3,4)
    plt.plot(time, j4, label='join4/positions')
    # plt.gca().set_title('J4')
    # plt.xlabel('time(s)')
    # plt.ylabel('joint radian')
    # plt.grid()

    # plt.subplot(2,3,5)
    plt.plot(time, j5, label='join5/positions')
    # plt.gca().set_title('J5')
    # plt.xlabel('time(s)')
    # plt.ylabel('joint radian')
    # plt.grid()
 
    # plt.subplot(2,3,6)
    plt.plot(time, j6, label='join6/positions')

    plt.xlabel('time (sec)', fontsize='18')
    plt.ylabel('joint position (rad)', fontsize='18')
    plt.gca().set_title('JointTrajectory', fontsize='24')
    plt.grid()
    plt.legend(loc='best')
    plt.tick_params(axis='x', direction='in', labelsize=12, width=1)
    plt.tick_params(axis='y', direction='in', labelsize=12, width=1)


    plt.figure(2)
    plt.ylim([-0.15, 0.15])
    plt.plot(time, j1_v, label='join1/velocity')
    plt.plot(time, j2_v, label='join2/velocity')
    plt.plot(time, j3_v, label='join3/velocity')
    plt.plot(time, j4_v, label='join4/velocity')
    plt.plot(time, j5_v, label='join5/velocity')
    plt.plot(time, j6_v, label='join6/velocity')
    plt.xlabel('time (sec)', fontsize='18')
    plt.ylabel('joint velocity (rad/s)', fontsize='18')
    # plt.gca().set_title('J6')
    plt.gca().set_title('JointTrajectory', fontsize='24')
    plt.grid()
    plt.legend(loc='best')
    plt.tick_params(axis='x', direction='in', labelsize=12, width=1)
    plt.tick_params(axis='y', direction='in', labelsize=12, width=1)
    plt.show()
    plt.tight_layout()


if __name__ == '__main__':
    listener()