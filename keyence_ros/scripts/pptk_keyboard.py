#!/usr/bin/env python2
# modified teleop_twist_key package
# wrriten by Ben Lee/HaeGu Lee
# Hanyang Univ. Robotics

from __future__ import print_function

import roslib
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import sys, select, termios, tty

movebindings ={
		#x,y,z,th
    'q':(1,0), #forward
    'w':(0,1), #backward
}

def getkey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('switch_flag', Int32, queue_size = 1)
    rospy.init_node('pptk_keyboard')
    x = 0

    try :
        while(1):
            print("press q for start pptk , stupdown for w")
            key = getkey()
            if key in movebindings.keys():
                x = movebindings[key][0]
                print("switch flag : ", x)
                pub.publish(x)
            else:
                x = 0
                if(key == '\x03'):  # \x03 = ^c
                    break	

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

