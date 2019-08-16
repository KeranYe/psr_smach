#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float32
from psr_msgs.msg import PSR_Drive

wheel_angle_left = 0
wheel_angle_right = 0
angular_speed_left = 0 # rad/s
angular_speed_right = 0 # rad/s
angular_acceleration_left = 0
angular_acceleration_right = 0
counter = 0.0



def talker():
    global wheel_angle_left, wheel_angle_right, angular_speed_left, angular_speed_right, angular_acceleration_left, angular_acceleration_right, counter
    pub = rospy.Publisher('/PSR/motors', PSR_Drive, queue_size=1)
    rospy.init_node('psudo_vel_controller', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    wheel_angle_left_init = math.pi/30.0# + math.pi/2
    wheel_angle_right_init = math.pi/30.0
    counter = 0
    while not rospy.is_shutdown():
	wheel_angle_left = wheel_angle_left_init + counter*math.pi/2
	wheel_angle_right = wheel_angle_right_init + counter*math.pi/2
	psr_msg = PSR_Drive()
	psr_msg.id = "psr-01"
	psr_msg.reset = False		
	psr_msg.theta_left_des = wheel_angle_left
	psr_msg.theta_right_des = wheel_angle_right
	psr_msg.omega_left_des = angular_speed_left
	psr_msg.omega_right_des = angular_speed_right
	psr_msg.alpha_left_des = angular_acceleration_left
	psr_msg.alpha_right_des = angular_acceleration_right
	psr_msg.duty_left_des = -0.7
	psr_msg.duty_right_des = 0.7
        rospy.loginfo('Publishing')
        pub.publish(psr_msg)
	
	rospy.loginfo('Publishing theta left = %f, theta right = %f', wheel_angle_left, wheel_angle_right)
	counter = counter + 1
	if counter > 3:
		counter = 0.0
        rate.sleep()
    if rospy.is_shutdown():
	psr_msg = PSR_Drive()
	psr_msg.id = "psr-01"
	psr_msg.reset = False		
	psr_msg.theta_left_des = 0
	psr_msg.theta_right_des = 0
	psr_msg.omega_left_des = 0
	psr_msg.omega_right_des = 0
	psr_msg.alpha_left_des = 0
	psr_msg.alpha_right_des = 0
	psr_msg.duty_left_des = 0
	psr_msg.duty_right_des = 0
        rospy.loginfo('Publishing')
        pub.publish(psr_msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	
        pass
