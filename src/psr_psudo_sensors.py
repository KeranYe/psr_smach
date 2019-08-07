#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('/PSR/sensors', Float32, queue_size=1)
    rospy.init_node('psudo_sensors', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
	msg = Float32()
	theta = 0
	msg.data = theta	
        rospy.loginfo('Publishing theta = %f', theta)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
