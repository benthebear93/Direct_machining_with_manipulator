#!/usr/bin/env python2

import sys
import rospy
from std_msgs.msg import String

class test_node(object):
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        print ("============ Printing robot state")

        rospy.Subscriber("chatter", String, callback)



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
    test_node = test_node()
    rospy.spin()