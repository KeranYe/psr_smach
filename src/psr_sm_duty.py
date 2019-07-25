#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Globle Variable
WheelTread = 1.0
portion = 0.3 # increment portion for each iteration in acceleration
accelTime = 0.15 # time or each iteration in acceleration, i.e, total time for acceleration is 2*accelTime. Total time = 0.3s.
################################################################################################
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToAccel','ShutDown'],
			     output_keys=['linear_vel_out','angular_vel_out','running_time_out','if_shutdown_out'])
	self.if_shutdown = 'y'
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
	while True:
		self.if_shutdown = raw_input("Shutdown State Machine?(y or n): ")
		if self.if_shutdown == 'y':
			return 'ShutDown'
		elif self.if_shutdown == 'n':
			userdata.running_time_out = float(raw_input("Enter running time(s): "))
			userdata.linear_vel_out = float(raw_input("Enter linear velocity(m/s): "))
			userdata.angular_vel_out = float(raw_input("Enter angulat velocity(rad/s): "))
			return 'GoToAccel'
		else:
			rospy.loginfo('Invalid Input')


######################################################################################################

# define sub-state machine Accelerate

# define state Vel2Duty
class Vel2Duty(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToAccel','NoAccel'],
                             input_keys=['linear_vel_in','angular_vel_in','running_time_in'],
                             output_keys=['LeftDuty','RightDuty'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Vel2Duty: Computing duty cycles')
	# Check if no running task, i.e, running_time_in = 0
	if userdata.running_time_in == 0:
		userdata.LeftDuty = 0
                userdata.RightDuty = 0
		return 'NoAccel'
        # Map linear and angular velocities to left and right duty cycles
	if userdata.angular_vel_in > 0:
		userdata.LeftDuty = userdata.linear_vel_in
		userdata.RightDuty = userdata.linear_vel_in + userdata.angular_vel_in * WheelTread
		return 'GoToAccel'
	else:
		userdata.RightDuty = userdata.linear_vel_in
                userdata.LeftDuty = userdata.linear_vel_in - userdata.angular_vel_in * WheelTread
		return 'GoToAccel'

# define state subPub
class subPub(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Delay','EndAccel'],
                             input_keys=['LeftDuty','RightDuty'],output_keys=['LeftDuty_out','RightDuty_out'])
        self.pub = rospy.Publisher('/turtlebot_teleop/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()
	self.counter = 0.0
#       rospy.init_node('talker', anonymous=True)

    def execute(self, userdata):
        userdata.LeftDuty_out = userdata.LeftDuty
	userdata.RightDuty_out = userdata.RightDuty
	rospy.loginfo('Executing state subPub')
	self.counter = self.counter + 1.0
	if self.counter > 2:
		self.counter = 0.0
		return 'EndAccel'
	else:
#		self.counter = self.counter + 1.0
        	if not rospy.is_shutdown():
            		self.vel_msg.linear.y = 0
            		self.vel_msg.linear.z = 0
            		self.vel_msg.angular.x = 0
            		self.vel_msg.angular.y = 0
            		self.vel_msg.linear.x = ((userdata.LeftDuty+userdata.RightDuty)/2.0) * portion * self.counter
            		self.vel_msg.angular.z = ((userdata.LeftDuty+userdata.RightDuty)/2.0) * portion * self.counter

            		while self.pub.get_num_connections() == 0:
               			continue

          		self.pub.publish(self.vel_msg)
            		rospy.loginfo('Accelerating!')
        		return 'Delay'
		else:
			self.vel_msg.linear.y = 0
                        self.vel_msg.linear.z = 0
                        self.vel_msg.angular.x = 0
                        self.vel_msg.angular.y = 0
                        self.vel_msg.linear.x = 0
                        self.vel_msg.angular.z = 0

                        while self.pub.get_num_connections() == 0:
                                continue

                        self.pub.publish(self.vel_msg)
                        rospy.loginfo('ROS Crushed!')
                        return 'EndAccel'




# define state subDelay
class subDelay(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoBack'])

    def execute(self, userdata):
        rospy.loginfo('Executing state subDelay')
#       userdata.running_time_out = 0
        now = rospy.get_time()
	# Acceleration time is 2*accelTime
        while (rospy.get_time() - now) < accelTime:
                continue
#        userdata.running_time_out = 0
#       runtime = rospy.Duration(userdata.running_time_in, 0)
#       rospy.sleep(runtime)
#        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'GoBack'


###################################################################################################


class PubVelTime(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['Wait','GoToIdle'],
                             input_keys=['LeftDuty','RightDuty','running_time_in'])
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
            self.vel_msg.linear.x = userdata.LeftDuty
#	    self.vel_msg.linear.x = 1
            self.vel_msg.angular.z = userdata.RightDuty

	    while self.pub.get_num_connections() == 0:
		continue
 
	    self.pub.publish(self.vel_msg)
	    rospy.loginfo('Robot Running!')
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
    sm.userdata.sm_left_duty = 0.0
    sm.userdata.sm_right_duty = 0.0
    sm.userdata.sm_lin_vel = 0.0
    sm.userdata.sm_ang_vel = 0.0
    sm.userdata.sm_run_time = 0.0
    sm.userdata.sm_if_shutdown = 'n'

    # Open the container sm
    with sm:
        # Add states to the container sm
	smach.StateMachine.add('IDLE', Idle(),
                               transitions={'GoToAccel':'SUBACCEL',
                                            'ShutDown':'End'},
                               remapping={'linear_vel_out':'sm_lin_vel',
                                          'angular_vel_out':'sm_ang_vel',
                                          'running_time_out':'sm_run_time',
					  'if_shutdown_out':'sm_if_shutdown'})


        # Creat Sub State machine: sm_accel
	sm_accel = smach.StateMachine(outcomes=['GoToPub'],
				      input_keys=['smaccel_lin_vel','smaccel_ang_vel','smaccel_run_time'],
				      output_keys=['smaccel_left_duty','smaccel_right_duty'])
	sm_accel.userdata.accel_left_duty = 0.0
	sm_accel.userdata.accel_right_duty = 0.0
#	sm_accel.userdata.smaccel_lin_vel = 0.0
#	sm_accel.userdata.accel_ang_vel = 0.0
#	sm_accel.userdata.accel_run_time = 0.0

	# Open the container sm_accel
	with sm_accel:
		
		smach.StateMachine.add('VEL2DUTY', Vel2Duty(),
                               transitions={'GoToAccel':'SUBPUB',
					    'NoAccel':'GoToPub'},
                               remapping={'linear_vel_in':'smaccel_lin_vel',
                                          'angular_vel_in':'smaccel_ang_vel',
					  'running_time_in':'smaccel_run_time',
#                                          'LeftDuty':'smaccel_left_duty',
#                                          'RightDuty':'smaccel_right_duty',
					  'LeftDuty':'accel_left_duty',
                                          'RightDuty':'accel_right_duty'})

		smach.StateMachine.add('SUBPUB', subPub(),
				transitions={'Delay':'SUBDELAY',
                                            'EndAccel':'GoToPub'},
				remapping={'LeftDuty':'accel_left_duty',
                                          'RightDuty':'accel_right_duty',
					  'LeftDuty_out':'smaccel_left_duty',
                                          'RightDuty_out':'smaccel_right_duty'})
		
		smach.StateMachine.add('SUBDELAY', subDelay(),
				transitions={'GoBack':'SUBPUB'})
#        sm_accel.userdata.smaccel_left_duty = sm_accel.userdata.accel_left_duty
#        sm_accel.userdata.smaccel_right_duty = sm_accel.userdata.accel_right_duty

	smach.StateMachine.add('SUBACCEL', sm_accel,
                           	transitions={'GoToPub':'PUB'},
				remapping={'smaccel_lin_vel':'sm_lin_vel',
                                          'smaccel_ang_vel':'sm_ang_vel',
					  'smaccel_run_time':'sm_run_time',
					  'smaccel_left_duty':'sm_left_duty',
					  'smaccel_right_duty':'sm_right_duty'})

		
	smach.StateMachine.add('PUB', PubVelTime(), 
                               transitions={'Wait':'WAIT', 
                                            'GoToIdle':'IDLE'},
                               remapping={'LeftDuty':'sm_left_duty',
                                          'RightDuty':'sm_right_duty',
					  'running_time_in':'sm_run_time'})
        
	smach.StateMachine.add('WAIT', Waiting(), 
                               transitions={'GoBack':'PUB'},
                               remapping={'running_time_in':'sm_run_time',
					  'running_time_out':'sm_run_time'})


    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
