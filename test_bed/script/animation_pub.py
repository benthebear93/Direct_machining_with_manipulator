#!/usr/bin/env python2
# license removed for brevity
import rospy
from std_msgs.msg import Int32

rospy.init_node('sender')
pub = rospy.Publisher('msg_send', Int32, queue_size = 10)

rate = rospy.Rate(10)

for i in range(1, 100+1):
    pub.publish(i)
    rate.sleep()