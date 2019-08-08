#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from psr_msgs.msg import PSR_Drive

def callback(psr_msg):
    #rospy.loginfo("id = %s, reset = %r", psr_msg.id, psr_msg.reset)
    rospy.loginfo("reset = %r", psr_msg.reset)
    rospy.loginfo("Wl = %4.2f, Wr = %4.2f", psr_msg.omega_left_des, psr_msg.omega_right_des)
    rospy.loginfo("al = %4.2f, ar = %4.2f", psr_msg.alpha_left_des, psr_msg.alpha_right_des)
        
def listener():
    rospy.init_node('psudo_bbbl', anonymous=True)

    rospy.Subscriber("/PSR/motors", PSR_Drive, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
