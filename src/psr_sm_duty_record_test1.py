#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import csv
import sys
import datetime

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped

# Globle Variable
posture = 'sprawl'
direction = 'right'
speed = 'fast'
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
			userdata.linear_vel_out = float(raw_input("Enter linear duty cycle(0<=,<=1): "))
			userdata.angular_vel_out = float(raw_input("Enter angular duty cycle(0<=,<=1]): "))
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
                             outcomes=['Wait','GoToSaveData'],
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
            return 'GoToSaveData'


# define state Waiting
class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['GoBack'],
                             input_keys=['running_time_in','linear_vel_in','angular_vel_in'],output_keys=['running_time_out','psr_data_out'])
	self.subscriber = rospy.Subscriber('/vicon/PSR/PSR', TransformStamped, self.callback)
#	rospy.spin()    

    def callback(self, data):
	# Retrieve data from message
	self.vicon_data = []
	self.vicon_data.append(data.header.seq)
	self.vicon_data.append(data.header.stamp.secs)
	self.vicon_data.append(data.header.stamp.nsecs)
	self.vicon_data.append(data.header.frame_id)
	self.vicon_data.append(data.child_frame_id)
	self.vicon_data.append(data.transform.translation.x)
	self.vicon_data.append(data.transform.translation.y)
	self.vicon_data.append(data.transform.translation.z)
	self.vicon_data.append(data.transform.rotation.x)
	self.vicon_data.append(data.transform.rotation.y)
	self.vicon_data.append(data.transform.rotation.z)
	self.vicon_data.append(data.transform.rotation.w)


    def execute(self, userdata):
        rospy.loginfo('Executing state Waiting')
	self.psr_data = []

	# Get Trial date and time
        self.TrialTimeVel = []
        self.TrialTimeVel.append(str(datetime.datetime.now()))

	# Get desired running time, linear and angular velocities(duty cycle)
	self.TrialTimeVel.append(userdata.running_time_in)
	self.TrialTimeVel.append(userdata.linear_vel_in)
	self.TrialTimeVel.append(userdata.angular_vel_in)

	self.psr_data.append(self.TrialTimeVel)

	# Record data from topic '/vicon/PSR/PSR' while waiting
	now = rospy.get_time()
	self.rate = rospy.Rate(100)
	while (rospy.get_time() - now)<userdata.running_time_in:
		self.psr_data.append(self.vicon_data)
		rospy.loginfo(self.vicon_data[5])
		self.rate.sleep()
#		continue
	userdata.psr_data_out = self.psr_data
	userdata.running_time_out = 0
        return 'GoBack'

####################################################################################################################        
# define state SaveData
class SaveData(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToIdle'],
                             input_keys=['psr_data_in'])

    def execute(self, userdata):
        rospy.loginfo('Saving Data now!')
	# Define file path
#	self.spring_coeff = raw_input("Spring Coeffient: ")
#	self.mount_pos = raw_input("Spring Mounting Position: ")
#	self.fixed_posture = raw_input("Fixed Posture(upright or sprawl): ")
#        self.direction = raw_input("Running Direction(straight, left or right): ")
#        self.speed_level = raw_input("Speed Level(fast or slow): ")
        self.trial_num = raw_input("Trial Number(01,02,...): ")
	self.filepath = '/home/keran/Documents/PSR_Trial_Data/test1-1/psr_'+posture+'_'+direction+'_'+speed+'_'+self.trial_num+'.txt'
	
	# Save data with csv
	with open(self.filepath,'w') as csvf:
		csv_writer = csv.writer(csvf)
		csv_writer.writerows(userdata.psr_data_in)

	return 'GoToIdle'

#################################################################################################################

def main():
    rospy.init_node('psr_state_machine', anonymous=True)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['End'])
    sm.userdata.sm_left_duty = 0.0
    sm.userdata.sm_right_duty = 0.0
    sm.userdata.sm_lin_vel = 0.0
    sm.userdata.sm_ang_vel = 0.0
    sm.userdata.sm_run_time = 0.0
    sm.userdata.sm_if_shutdown = 'n'
    sm.userdata.sm_psr_data = []
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

	smach.StateMachine.add('SUBACCEL', sm_accel,
                           	transitions={'GoToPub':'PUB'},
				remapping={'smaccel_lin_vel':'sm_lin_vel',
                                          'smaccel_ang_vel':'sm_ang_vel',
					  'smaccel_run_time':'sm_run_time',
					  'smaccel_left_duty':'sm_left_duty',
					  'smaccel_right_duty':'sm_right_duty'})

		
	smach.StateMachine.add('PUB', PubVelTime(), 
                               transitions={'Wait':'WAIT', 
                                            'GoToSaveData':'SAVE'},
                               remapping={'LeftDuty':'sm_left_duty',
                                          'RightDuty':'sm_right_duty',
					  'running_time_in':'sm_run_time'})
        
	smach.StateMachine.add('WAIT', Waiting(), 
                               transitions={'GoBack':'PUB'},
                               remapping={'running_time_in':'sm_run_time',
					  'linear_vel_in':'sm_lin_vel',
					  'angular_vel_in':'sm_ang_vel',
					  'running_time_out':'sm_run_time',
					  'psr_data_out':'sm_psr_data'})

	smach.StateMachine.add('SAVE', SaveData(),
                               transitions={'GoToIdle':'IDLE'},
                               remapping={'psr_data_in':'sm_psr_data'})

	
    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
