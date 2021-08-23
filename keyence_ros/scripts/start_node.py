#!/usr/bin/env python2
""" 
start node is for testing z axis platform. each number will start, stop and change direction
arduino <-> rosserial_server <-> pc

Author : Haegu Lee

"""

import rospy
import numpy as np
import math
from std_msgs.msg import Float64
from std_msgs.msg import Int16
import time


def start_or_stop():
    rate = rospy.Rate(100) #10hz = 0.1s = 100ms
    while not rospy.is_shutdown():
        print("===================================")
        print("== 010 : start z axis correction ==")
        print("== 000 : stop  z axis correction ==")
        print("== 110 :     reset go forward    ==")
        print("== 111 :     reset go backward   ==")
        print("===================================")
        flag3, flag, flag2 = map(int, raw_input("put command : ").split())
        pub3.publish(flag3)
        pub.publish(flag)
        pub2.publish(flag2)
        rate.sleep()

if __name__=='__main__':
    rospy.init_node('start_button', anonymous=True)
    pub = rospy.Publisher('start_button', Int16, queue_size =10)
    pub2 = rospy.Publisher('go_back_button', Int16, queue_size = 10)
    pub3 = rospy.Publisher('reset_button', Int16, queue_size = 10)
    start_or_stop()
