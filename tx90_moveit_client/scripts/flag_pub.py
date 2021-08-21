#!/usr/bin/env python2
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    rospy.init_node('flag', anonymous=True)
    pub = rospy.Publisher('process_flag', Int32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    process_flag =1
    while not rospy.is_shutdown():
        pub.publish(process_flag)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass