#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float32
from psr_msgs.msg import PSR_Drive
from psr_msgs.msg import Puppy_pos

pos_front_left_upper = 0.0
pos_front_left_lower = 0.0

pos_rear_left_upper = 0.0
pos_rear_left_lower = 0.0

pos_front_right_upper = 0.0
pos_front_right_lower = 0.0

pos_rear_right_upper = 0.0
pos_rear_right_lower = 0.0

counter = 0.0
angle = 0.0


def talker():
    global pos_front_left_upper, pos_front_left_lower, pos_rear_left_upper, pos_rear_left_lower, pos_front_right_upper, pos_front_right_lower, pos_rear_right_upper, pos_rear_right_lower, counter, angle
    pub = rospy.Publisher('/PUPPY/pos', Puppy_pos, queue_size=1)
    rospy.init_node('psudo_pos_controller', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    counter = 0
    while not rospy.is_shutdown():

	pos_front_left_upper = math.sin(angle)	
	
	puppy_msg = Puppy_pos()
	puppy_msg.id = "puppy-01"
	puppy_msg.reset = False		
	puppy_msg.pos_front_left_upper_des = pos_front_left_upper
	puppy_msg.pos_front_left_lower_des = pos_front_left_lower
	puppy_msg.pos_rear_left_upper_des = pos_rear_left_upper
	puppy_msg.pos_rear_left_lower_des = pos_rear_left_lower
	puppy_msg.pos_front_right_upper_des = pos_front_right_upper
	puppy_msg.pos_front_right_lower_des = pos_front_right_lower
	puppy_msg.pos_rear_right_upper_des = pos_rear_right_upper
	puppy_msg.pos_rear_right_lower_des = pos_rear_right_lower

        rospy.loginfo('Publishing')
        pub.publish(puppy_msg)
		
	angle = angle + math.pi/2.0	
	counter = counter + 1
	if counter > 4.0:
		counter = 1.0
		angle = math.pi/2.0
        rate.sleep()

    if rospy.is_shutdown():
	puppy_msg = Puppy_pos()
	puppy_msg.id = "puppy-01"
	puppy_msg.reset = False		
	puppy_msg.pos_front_left_upper_des = 0.0
	puppy_msg.pos_front_left_lower_des = 0.0
	puppy_msg.pos_rear_left_upper_des = 0.0
	puppy_msg.pos_rear_left_lower_des = 0.0
	puppy_msg.pos_front_right_upper_des = 0.0
	puppy_msg.pos_front_right_lower_des = 0.0
	puppy_msg.pos_rear_right_upper_des = 0.0
	puppy_msg.pos_rear_right_lower_des = 0.0

        rospy.loginfo('Shutting Down')
        pub.publish(puppy_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	
        pass
