#! /usr/bin/env python

import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import psr_smach.msg

def psr_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('psr', psr_smach.msg.PSRAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = psr_smach.msg.PSRGoal(duration=5.0, angular_speed=1.0, linear_speed=1.0)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('psr_client_type1')
        result = psr_client()
        print("Succeed! angular speed:", result.angular_speed)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
