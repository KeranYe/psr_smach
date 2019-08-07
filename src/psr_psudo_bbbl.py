#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(psr_msg):
    rospy.loginfo("Wl = %f, Wr = %f", psr_msg.angular.x, psr_msg.angular.y)
    
def listener():
    rospy.init_node('psudo_bbbl', anonymous=True)

    rospy.Subscriber("/PSR/motors", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
