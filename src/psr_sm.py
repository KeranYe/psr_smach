#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#global vel_msg
# define state PubVelTime
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToPub','ShutDown'],
#                             input_keys=['linear_vel_in','angular_vel_in','running_time_in','if_shutdown_in'],
			     output_keys=['linear_vel_out','angular_vel_out','running_time_out','if_shutdown_out'])
#        self.pub = rospy.Publisher('/turtlebot_teleop/cmd_vel', Twist, queue_size=1)
#        self.vel_msg = Twist()
#       rospy.init_node('talker', anonymous=True)
	self.if_shutdown = 'y'
    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
	self.if_shutdown = raw_input("Shutdown State Machine?(y or n): ")
#	userdata.if_shutdown_out = userdata.if_shutdown_in
	if self.if_shutdown == 'y':
		return 'ShutDown'
	else:
		userdata.running_time_out = float(raw_input("Enter running time(s): "))
		userdata.linear_vel_out = float(raw_input("Enter linear velocity(m/s): "))
		userdata.angular_vel_out = float(raw_input("Enter angulat velocity(rad/s): "))
#		userdata.running_time_out =  userdata.running_time_in
#		userdata.linear_vel_out = userdata.linear_vel_in
#		userdata.angular_vel_out = userdata.angular_vel_in
		return 'GoToPub'


class PubVelTime(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['Wait','GoToIdle'],
                             input_keys=['linear_vel_in','angular_vel_in','running_time_in'],
                             output_keys=['running_time_out'])
	self.pub = rospy.Publisher('/turtlebot_teleop/cmd_vel', Twist, queue_size=1)
	self.vel_msg = Twist()
#	rospy.init_node('talker', anonymous=True)

    def execute(self, userdata):
        rospy.loginfo('Executing state PubVelTime')
	if not rospy.is_shutdown():
#	    self.vel_msg.linear.x = userdata.linear_vel_in
#	    self.vel_msg.angular.z = userdata.angular_vel_in
	    self.vel_msg.linear.y = 0
    	    self.vel_msg.linear.z = 0
    	    self.vel_msg.angular.x = 0
    	    self.vel_msg.angular.y = 0
#	    self.pub.publish(vel_msg)
        if userdata.running_time_in > 0:
            self.vel_msg.linear.x = userdata.linear_vel_in
#	    self.vel_msg.linear.x = 1
            self.vel_msg.angular.z = userdata.angular_vel_in

	    while self.pub.get_num_connections() == 0:
		continue
 
	    self.pub.publish(self.vel_msg)
	    rospy.loginfo('Message Sent!')
            userdata.running_time_out = userdata.running_time_in
            return 'Wait'
        else:
	    self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = 0
            self.pub.publish(self.vel_msg)
            return 'GoToIdle'


# define state Waiting
class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['GoBack'],
                             input_keys=['running_time_in'],output_keys=['running_time_out'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Waiting')
#	userdata.running_time_out = 0
	now = rospy.get_time()
	while (rospy.get_time() - now)<userdata.running_time_in:
		continue
	userdata.running_time_out = 0
#	runtime = rospy.Duration(userdata.running_time_in, 0)
#	rospy.sleep(runtime)
#        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'GoBack'
        




def main():
    rospy.init_node('psr_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['End'])
    sm.userdata.sm_lin_vel = 0
    sm.userdata.sm_ang_vel = 0
    sm.userdata.sm_run_time = 0
    sm.userdata.sm_if_shutdown = 'n'

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add('IDLE', Idle(),
                               transitions={'GoToPub':'PUB',
                                            'ShutDown':'End'},
                               remapping={'linear_vel_out':'sm_lin_vel',
                                          'angular_vel_out':'sm_ang_vel',
                                          'running_time_out':'sm_run_time',
					  'if_shutdown_out':'sm_if_shutdown'})
        smach.StateMachine.add('PUB', PubVelTime(), 
                               transitions={'Wait':'WAIT', 
                                            'GoToIdle':'IDLE'},
                               remapping={'linear_vel_in':'sm_lin_vel',
                                          'angular_vel_in':'sm_ang_vel',
					  'running_time_in':'sm_run_time',
					  'running_time_out':'sm_run_time'})
        smach.StateMachine.add('WAIT', Waiting(), 
                               transitions={'GoBack':'PUB'},
                               remapping={'running_time_in':'sm_run_time','running_time_out':'sm_run_time'})


    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
