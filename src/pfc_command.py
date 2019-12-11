#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from psr_msgs.msg import pfc

def talker():
    pub = rospy.Publisher('/pfc/demo', pfc, queue_size=1)
    rospy.init_node('pfc_command', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
	  msg = pfc()
	  top_servo_pwm = 1500
    bottom_servo_pwm = 1500
    led_on = 0
	  msg.top_pwm_width = top_servo_pwm
    msg.bottom_pwm_width = bottom_servo_pwm
    msg.uv_led_on = led_on
  
    rospy.loginfo('top_servo_pwm = %d, bottom_servo_pwm = %d', top_servo_pwm, bottom_servo_pwm)
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
