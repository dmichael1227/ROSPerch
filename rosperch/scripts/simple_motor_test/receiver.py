#!/usr/bin/env python3
# Decription:
# This program takes user input commands and sends them
# to a seperate program that deals with making the motors
# on the ROSPerch either turn on or off.
# Adapted Sources:
# Motor command code is adapted from the UTAP 2020
# code at https://github.com/jdicecco/UTAP/
# ROS talker/listener node code based on tutorials at:
# https://github.com/ros/ros_tutorials
# License:
# Software License Agreement (GPLv3 License)
# Find the full agreement at https://github.com/amichael1227/ROSPerch/blob/master/LICENSE

# Imports the necessary libraries
import rospy
import subprocess
from std_msgs.msg import Bool

# Defines the path of the code that moves the motors
FILE_PATH = '/home/ubuntu/catkin_ws/src/rosperch/scripts/simple_motor_test/motor_code.py'

# Callback funtion
def callback(data):
    rospy.loginfo('I heard %s', data.data) # Log what is heard
#    subprocess.call(['sudo','python3', FILE_PATH,'%s' % data.data]) # Call motor_code.py when receiving a message
    subprocess.call(['python3', FILE_PATH,'%s' % data.data]) # Call motor_code.py when receiving a message
# Listener node function
def listener():
    # Set up the listener node
    rospy.init_node('listener', anonymous=True) 
    rospy.Subscriber('motorstuff', Bool, callback) # subscribe to ledstuff topic

    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()