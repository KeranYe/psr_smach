#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import csv
import sys
import datetime
import actionlib

# Brings in the messages used by the PSR action, including the goal message and the result message.
import psr_smach.msg

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped

# Globle Variable
duration = 0.0
angular_velocity_des = 0.0
linear_velocity_des = 0.0

if_collect_data = False # Flag for collecting data
if_start_client = False # Flag for starting data

psr_data = []

posture = 'sprawl'
direction = 'right'
speed = 'slow'
accel_loop_num = 2
WheelTread = 1.0
portion = 0.3 # increment portion for each iteration in acceleration
accelTime = 0.15 # time or each iteration in acceleration, i.e, total time for acceleration is 4*accelTime. Total time = 1.6s.
################################################################################################
# define state 'Idle'
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				     outcomes=['Go','ShutDown'])
		self.if_shutdown = 'y'
    
	def execute(self, userdata):
		global if_collect_data, if_start_client

		rospy.loginfo('Executing state: Idle')

		# Reset flags
		if_collect_data = False # Flag for collecting data
		if_start_client = False # Flag for starting data
		
		while True:
			self.if_shutdown = raw_input("Shutdown State Machine?(y or n): ")

			if self.if_shutdown == 'y':
				return 'ShutDown'
			elif self.if_shutdown == 'n':
				return 'Go'
			else:
				rospy.loginfo('Invalid Input! Enter again.')

######################################################################################################
# define state 'Initial'
class Initial(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				     outcomes=['Go','Cancel'])
		self.if_right = 'y'
    
	def execute(self, userdata):
		global duration, angular_velocity_des, linear_velocity_des
		
		rospy.loginfo('Executing state: Initial')
		
		# Initialize test parameters
		rospy.loginfo('Initialize some parameters for test!')

		duration = float(raw_input("Enter test duration (s): "))
		angular_velocity_des = float(raw_input("Enter desired angular velocity (rad/s): "))
		linear_velocity_des = float(raw_input("Enter desired linear velocity (rad/s): "))
		
		rospy.loginfo('Initialize some parameters for test!')
		
		# Double check
		while True:
			self.if_right = raw_input("Right Inputs? (y or n)")
			if self.if_right == 'y':
				return 'Go'
			elif self.if_right == 'n':
				rospy.loginfo('Head back to Idle!')
				return 'Cancel'
			else:
				rospy.loginfo('Invalid Input! Enter again.')
####################################################################################################################
# define state 'PSR_client'
class PSR_client(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				     outcomes=['Completed'])
		self.if_shutdown = 'y'
    
	def psr_client(self):
		global if_collect_data, if_start_client
		
		# Wait till client sends out a goal message
		while not if_start_client:
			pass
		
		# Creates the SimpleActionClient, passing the type of the action
		# (PSRAction) to the constructor.
		self.client = actionlib.SimpleActionClient('psr_type1', psr_smach.msg.PSRAction)

		# Waits until the action server has started up and started
		# listening for goals.
		self.client.wait_for_server()
		rospy.loginfo('Server connected!!')

		# Creates a goal to send to the action server.
		self.goal = psr_smach.msg.PSRGoal(duration=duration, angular_speed=angular_velocity_des, linear_speed=linear_velocity_des)
		
		# Set flag to start collecting data
		if_collect_data = True

		# Sends the goal to the action server.
		self.client.send_goal(self.goal)
		rospy.loginfo('Goal sent!!')

		# Waits for the server to finish performing the action.
		self.client.wait_for_result()

		# Set flag to stop collecting data
		if_collect_data = False

		# Prints out the result of executing the action
		return self.client.get_result()  # A PSRResult

	def execute(self, userdata):
		rospy.loginfo('Executing state: PSR_client')
		
		self.result = self.psr_client()
		rospy.loginfo("Action Succeed!")
		
		return 'Completed'

####################################################################################################################
# define state 'Data_collect'
class Data_collect(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				     outcomes=['Completed'])
		self.subscriber = rospy.Subscriber('/vicon/PSR/PSR', TransformStamped, self.callback)
		
    
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
		global if_collect_data, if_start_client, duration, angular_velocity_des, linear_velocity_des, psr_data
		
		rospy.loginfo('Executing state: Data_collect')
		
		# Record test info
        	self.Test_Info = []
        	self.Test_Info.append(str(datetime.datetime.now()))
        	self.Test_Info.append('Running Time: ')
		self.Test_Info.append(duration)
		self.Test_Info.append('Desired linear velocity: ')
        	self.Test_Info.append(linear_velocity_des)
		self.Test_Info.append('Desired angular velocity: ')
        	self.Test_Info.append(angular_velocity_des)
		
		#psr_data = []
        	psr_data.append(self.Test_Info)
		
		# Set flag to start client
		if_start_client = True
		
		# Wait till client sends out a goal message
		while not if_collect_data:
			pass
		
		rospy.loginfo('Collect Data!')
		# Collect data!		
		self.rate = rospy.Rate(100)
		while if_collect_data:
			psr_data.append(self.vicon_data)
			rospy.loginfo(self.vicon_data[5])
			self.rate.sleep()
		
		#userdata.psr_data_out = self.psr_data
		
		return 'Completed'

####################################################################################################################
# define state SaveData
class Save_Data(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				     outcomes=['Done'])

	def execute(self, userdata):
		global psr_data
		rospy.loginfo('Saving Data now!')
		# Define file path
		#self.spring_coeff = raw_input("Spring Coeffient: ")
		#self.mount_pos = raw_input("Spring Mounting Position: ")
		#self.fixed_posture = raw_input("Fixed Posture(upright or sprawl): ")
		#self.direction = raw_input("Running Direction(straight, left or right): ")
		#self.speed_level = raw_input("Speed Level(fast or slow): ")
		self.trial_num = raw_input("Trial Number(01,02,...): ")
		#self.filepath = '/home/keranye/Documents/PSR_Trial_Data/test1/psr_'+posture+'_'+direction+'_'+speed+'_'+self.trial_num+'.txt' # path for minix
		self.filepath = '/home/keran/Documents/PSR_Trial_Data/test_example/psr_'+posture+'_'+direction+'_'+speed+'_'+self.trial_num+'.txt' # path for CE-CERT NUC

		# Save data with csv
		with open(self.filepath,'w') as csvf:
			csv_writer = csv.writer(csvf)
			csv_writer.writerows(psr_data)
		
		psr_data = []

		return 'Done'

#################################################################################################################

def main():
	rospy.init_node('psr_state_machine', anonymous=True)

	# Create a SMACH state machine 'sm'
	sm = smach.StateMachine(outcomes=['Exit'])
	
	# Local variables for sm
	#sm.userdata.sm_psr_data = [1]

	# Open the container sm
	with sm:
        
		# Add states to the container 'sm'
		smach.StateMachine.add('IDLE', Idle(),
				       transitions={'Go':'INITIAL',
						    'ShutDown':'Exit'})		
		
		smach.StateMachine.add('INITIAL', Initial(), 
				       transitions={'Go':'CON', 
						    'Cancel':'IDLE'})

        	# Create a sub SMACH state machine 'sm_con'
		sm_con = smach.Concurrence(outcomes=['Completed','Aborted'],
					   default_outcome='Aborted',
					   outcome_map={'Completed':
								  { 'PSR_CLIENT':'Completed',
								    'DATA_COLLECT':'Completed'}})		
		#sm_con.userdata.sm_con_psr_data = [2]
		# Add states to the container 'sm_con'
		with sm_con:
		
			smach.Concurrence.add('PSR_CLIENT', PSR_client())
			smach.Concurrence.add('DATA_COLLECT', Data_collect())

		smach.StateMachine.add('CON', sm_con,
					transitions={'Completed':'SAVE',
						     'Aborted':'SAVE'})

		# Add states to the container 'sm'
		smach.StateMachine.add('SAVE', Save_Data(),
				       transitions={'Done':'IDLE'})
	
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_psr', sm, '/SM_ROOT')
	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()

"""
		smach.StateMachine.add('SAVE', SaveData(),
				       transitions={'GoToIdle':'IDLE'},
				       remapping={'psr_data_in':'sm_psr_data'})
"""
	



"""
# Globle Variable
posture = 'sprawl'
direction = 'right'
speed = 'slow'
accel_loop_num = 2
WheelTread = 1.0
portion = 0.3 # increment portion for each iteration in acceleration
accelTime = 0.15 # time or each iteration in acceleration, i.e, total time for acceleration is 4*accelTime. Total time = 1.6s.
################################################################################################
# define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToAccel','ShutDown'],
			     output_keys=['linear_vel_out','angular_vel_out','running_time_out','if_shutdown_out','psr_data_out'])
	self.if_shutdown = 'y'
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
	while True:
		self.if_shutdown = raw_input("Shutdown State Machine?(y or n): ")
		if self.if_shutdown == 'y':
			return 'ShutDown'
		elif self.if_shutdown == 'n':
			self.running_time = float(raw_input("Enter running time(s): "))
			self.linear_vel = float(raw_input("Enter linear duty cycle(0<=,<=1): "))
			self.angular_vel = float(raw_input("Enter angular duty cycle(0<=,<=1]): "))
			userdata.running_time_out = self.running_time
			userdata.linear_vel_out = self.linear_vel
			userdata.angular_vel_out = self.angular_vel
			# Create data file and write 
			self.psr_data = []

		        # Get Trial date and time
        		self.Test_Info = []
        		self.Test_Info.append(str(datetime.datetime.now()))

       			# Get desired running time, linear and angular velocities(duty cycle)
        		self.Test_Info.append('Running Time: ')
			self.Test_Info.append(self.running_time)
			self.Test_Info.append('Linear Duty cycle: ')
        		self.Test_Info.append(self.linear_vel)
			self.Test_Info.append('Angular Duty cycle: ')
        		self.Test_Info.append(self.angular_vel)

        		self.psr_data.append(self.Test_Info)
			userdata.psr_data_out = self.psr_data

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
	if self.counter > accel_loop_num:
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
                             outcomes=['GoBack'],input_keys=['psr_data_in'],output_keys=['psr_data_out'])
	self.subscriber = rospy.Subscriber('/vicon/PSR/PSR', TransformStamped, self.callback)

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
        rospy.loginfo('Executing state subDelay')
	self.psr_data = userdata.psr_data_in
        now = rospy.get_time()
        self.rate = rospy.Rate(100)
        while (rospy.get_time() - now) < accelTime:
                self.psr_data.append(self.vicon_data)
                rospy.loginfo(self.vicon_data[5])
                self.rate.sleep()
#               continue
        userdata.psr_data_out = self.psr_data
	# Acceleration time is 2*accelTime        
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
                             input_keys=['running_time_in','linear_vel_in','angular_vel_in','psr_data_in'],output_keys=['running_time_out','psr_data_out'])
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
	self.psr_data = userdata.psr_data_in
#	self.psr_data = []

	# Get Trial date and time
#       self.Test_Info = []
#        self.Test_Info.append(str(datetime.datetime.now()))

	# Get desired running time, linear and angular velocities(duty cycle)
#	self.Test_Info.append(userdata.running_time_in)
#	self.Test_Info.append(userdata.linear_vel_in)
#	self.Test_Info.append(userdata.angular_vel_in)

#	self.psr_data.append('acceleration stop!!!')

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
	self.filepath = '/home/keran/Documents/PSR_Trial_Data/fail/psr_'+posture+'_'+direction+'_'+speed+'_'+self.trial_num+'.txt'
	
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
					  'if_shutdown_out':'sm_if_shutdown',
					  'psr_data_out':'sm_psr_data'})


        # Creat Sub State machine: sm_accel
	sm_accel = smach.StateMachine(outcomes=['GoToPub'],
				      input_keys=['smaccel_lin_vel','smaccel_ang_vel','smaccel_run_time','smaccel_psr_data_in'],
				      output_keys=['smaccel_left_duty','smaccel_right_duty','smaccel_psr_data_out'])
	sm_accel.userdata.accel_left_duty = 0.0
	sm_accel.userdata.accel_right_duty = 0.0
	sm_accel.userdata.accel_psr_data = []
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
				transitions={'GoBack':'SUBPUB'},
				remapping={'psr_data_in':'smaccel_psr_data_in',
                                          'psr_data_out':'smaccel_psr_data_out'})

	smach.StateMachine.add('SUBACCEL', sm_accel,
                           	transitions={'GoToPub':'PUB'},
				remapping={'smaccel_lin_vel':'sm_lin_vel',
                                          'smaccel_ang_vel':'sm_ang_vel',
					  'smaccel_run_time':'sm_run_time',
					  'smaccel_psr_data_in':'sm_psr_data',
					  'smaccel_left_duty':'sm_left_duty',
					  'smaccel_right_duty':'sm_right_duty',
					  'smaccel_psr_data_out':'sm_psr_data'})

		
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
					  'psr_data_in':'sm_psr_data',
					  'running_time_out':'sm_run_time',
					  'psr_data_out':'sm_psr_data'})

	smach.StateMachine.add('SAVE', SaveData(),
                               transitions={'GoToIdle':'IDLE'},
                               remapping={'psr_data_in':'sm_psr_data'})

	
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_psr', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
"""

