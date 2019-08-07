#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped

def talker():
    pub = rospy.Publisher('/vicon/PSR/PSR', TransformStamped, queue_size=100)
    rospy.init_node('psudo_VICON_bridge', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
	msg = TransformStamped()
	msg.header.seq = int(1)
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = str("id001") 
	msg.child_frame_id = str("id002") 
	msg.transform.translation.x = float(1)
	msg.transform.translation.y = float(2)
	msg.transform.translation.z = float(3)
	msg.transform.rotation.x = float(4)
	msg.transform.rotation.y = float(5)
	msg.transform.rotation.z = float(6)
	msg.transform.rotation.w = float(7)

        rospy.loginfo('Publishing info')
	
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
