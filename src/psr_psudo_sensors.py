#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('/PSR/sensors', Float32, queue_size=1)
    rospy.init_node('psudo_sensors', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
	theta = 0
        rospy.loginfo('Publishing theta = %f', theta)
        pub.publish(theta)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
