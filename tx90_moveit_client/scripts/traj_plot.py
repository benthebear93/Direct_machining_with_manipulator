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
    time = []
    for i in range(0,len(msg.points)):
        j1.append(msg.points[i].positions[0])
        j2.append(msg.points[i].positions[1])
        j3.append(msg.points[i].positions[2])
        j4.append(msg.points[i].positions[3])
        j5.append(msg.points[i].positions[4])
        j6.append(msg.points[i].positions[5])
        time.append(0.5*i)
    plt.subplot(2,3,1)
    plt.plot(time, j1)
    plt.gca().set_title('J1')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.subplot(2,3,2)
    plt.plot(time, j2)
    plt.gca().set_title('J2')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.subplot(2,3,3)
    plt.plot(time, j3)
    plt.gca().set_title('J3')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.subplot(2,3,4)
    plt.plot(time, j4)
    plt.gca().set_title('J4')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.subplot(2,3,5)
    plt.plot(time, j5)
    plt.gca().set_title('J5')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.subplot(2,3,6)
    plt.plot(time, j6)
    plt.gca().set_title('J6')
    plt.xlabel('time(s)')
    plt.ylabel('joint radian')
    plt.grid()

    plt.show()
    plt.tight_layout()


if __name__ == '__main__':
    listener()