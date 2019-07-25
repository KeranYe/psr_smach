#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

lin_scale = 0.8
ang_scale = 0.2

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
##################################################
# Mode 1: Linear velocity will not change
#    if data.axes[2] > 0:
#    	twist.linear.x = lin_scale * data.axes[1] + ang_scale * data.axes[2]
#    	twist.angular.z = lin_scale * data.axes[1]
#    else:
#	twist.angular.z = lin_scale * data.axes[1] - ang_scale * data.axes[2]
#        twist.linear.x = lin_scale * data.axes[1]
#    pub.publish(twist)
######################################################
# Mode 2: Linear velocity will change
    twist.linear.x = lin_scale * data.axes[1] + ang_scale * data.axes[2]
    twist.angular.z = lin_scale * data.axes[1] - ang_scale * data.axes[2]
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('/turtlebot_teleop/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()
