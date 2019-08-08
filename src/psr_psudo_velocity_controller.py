#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float32
from psr_smach.msg import PSR_Drive

wheel_angle_left = 0
wheel_angle_right = 0
angular_speed_left = math.pi # rad/s
angular_speed_right = math.pi # rad/s
angular_acceleration_left = 0
angular_acceleration_right = 0


def talker():
    pub = rospy.Publisher('/PSR/motors', PSR_Drive, queue_size=1)
    rospy.init_node('psudo_vel_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	psr_msg = PSR_Drive()
	psr_msg.id = "psr-01"
	psr_msg.reset = True		
	psr_msg.theta_left_des = wheel_angle_left
	psr_msg.theta_right_des = wheel_angle_right
	psr_msg.omega_left_des = angular_speed_left
	psr_msg.omega_right_des = angular_speed_right
	psr_msg.alpha_left_des = angular_acceleration_left
	psr_msg.alpha_right_des = angular_acceleration_right
	psr_msg.duty_left_des = 0
	psr_msg.duty_right_des = 0
        rospy.loginfo('Publishing')
        pub.publish(psr_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
