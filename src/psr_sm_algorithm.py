#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import csv
import sys
import datetime

from smach_ros import SimpleActionState
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped

# Globle Variable
posture = 'sprawl'
direction = 'right'
speed = 'slow'
accel_loop_num = 2
WheelTread = 1.0
portion = 0.3 # increment portion for each iteration in acceleration
accelTime = 0.15 # time or each iteration in acceleration, i.e, total time for acceleration is 4*accelTime. Total time = 1.6s.
################################################################################################
# define state: Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToInit','ShutDown'])
	self.if_shutdown = 'y'
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
	while True:
		self.if_shutdown = raw_input("Shutdown State Machine?(y or n): ")
		if self.if_shutdown == 'y':
			return 'ShutDown'
		elif self.if_shutdown == 'n':
			return 'GoToInit'
		else:
			rospy.loginfo('Invalid Input! Enter again!')


######################################################################################################
# define state: Initialization
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
			     outcomes=['StartTest'],
			     output_keys=['test_info'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialization')
	while True:
		# Empty test info
		userdata.test_info = []
		userdata.test_info.append(str(datetime.datetime.now())) # Get test date and time
		#userdata.test_info.append('Keran Ye') # Get researcher name
		userdata.test_info.append(str(raw_input("Enter researcher name: "))) # Get researcher name
		
		# Double check info
		self.if_shutdown = raw_input("Right infomation?(y or n): ")
		if self.if_shutdown == 'y':
			return 'StartTest'
		else:
			rospy.loginfo('Wrong infomation! Enter again!')

####################################################################################################################        
# define state: SaveData
class SaveData(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['GoToIdle'],
                             input_keys=['data_saver_in'])

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
		csv_writer.writerows(userdata.data_saver_in)

	return 'GoToIdle'

#################################################################################################################
# define state: PSR_Test
class PSR_Test(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

#################################################################################################################
# define state: Data_Collector
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'




#################################################################################################################
# define main function
def main():
	# 1. Create ROS node
	rospy.init_node('psr_smach')

	# 2. Create the top level SMACH state machine
	sm_top = smach.StateMachine(outcomes=['Exit'])
	sm.userdata.sm_test_info = []

	# 2.1 Open the container
	with sm_top:
	
		# 2.1.1 Add state 'Idle'
		smach.StateMachine.add('IDLE', Idle(),
					transitions={'GoToInit':'INIT',
						     'ShutDown':'Exit'})
		# 2.1.2 Add state 'Initialization'
		smach.StateMachine.add('INIT', Initialization(),
					transitions={'StartTest':'TEST'},
					remapping={'test_info':'sm_test_info'})

		# 2.1.3 Add the sub SMACH state machine
		# a. Create Concurrent Container
		sm_test = smach.Concurrence(outcomes=['Completed','Aborted'],
					default_outcome='Aborted',
					outcome_map={'Completed':{ 'PSR_TEST':'succeeded','DATA_COLLECTOR':'succeeded'}})

		# b. Open the container
		with sm_test:
            
			# b.1 Add SimpleActionState 'PSR_Test' to the container
			def psr_goal_cb(userdata, goal):
				psr_goal = PSRGoal()
				psr_goal.content1 = userdata.psr_input.content1
				psr_goal.content2 = userdata.psr_input.content2
				return psr_goal
			
			def psr_result_cb(userdata, status, result):
				if status == GoalStatus.SUCCEEDED:
				userdata.psr_output = result
				return 'succeeded'

			smach.Concurrence.add('PSR_TEST',
					SimpleActionState('action_server_namespace',
                                        		  PSRAction,
                                        		  goal_cb=psr_goal_cb,
							  result_cb=psr_result_cb,
                                        		  input_keys=['psr_input'],
							  output_keys=['psr_output'])
                      			#transitions={'succeeded':'APPROACH_PLUG'},
                      			remapping={'psr_input':'userdata_psr_input',
						   'psr_output':'userdata_psr_output'})

			# b.2 Add SimpleActionState 'Data_Collector' to the container
			def collector_goal_cb(userdata, goal):
				collector_goal = COLLECTORGoal()
				collector_goal.content1 = userdata.collector_input.content1
				collector_goal.content2 = userdata.collector_input.content2
				return collector_goal
			
			def collector_result_cb(userdata, status, result):
				if status == GoalStatus.SUCCEEDED:
				userdata.collector_output = result
				return 'succeeded'

			smach.Concurrence.add('COLLECTOR_TEST',
					SimpleActionState('action_server_namespace',
                                        		  COLLECTORAction,
                                        		  goal_cb=collector_goal_cb,
							  result_cb=collector_result_cb,
                                        		  input_keys=['collector_input'],
							  output_keys=['collector_output'])
                      			#transitions={'succeeded':'APPROACH_PLUG'},
                      			remapping={'collector_input':'userdata_collector_input',
						   'collector_output':'userdata_collector_output'})

		# c. Add container 'TEST'
		smach.StateMachine.add('TEST', sm_test,
					transitions={'Aborted':'IDLE',
						     'Completed':'SAVE_DATA'})
		
		# 2.1.4 Add state 'SaveData'
		smach.StateMachine.add('SAVE', SaveData(),
					transitions={'GoToIdle':'IDLE'},
					remapping={'data_save_in':'userdata_collector_output'})
		

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_psr', sm, '/SM_ROOT')
	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

"""
####################################################################################
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
"""

if __name__ == '__main__':
    main()


