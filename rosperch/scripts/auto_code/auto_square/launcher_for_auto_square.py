#!/usr/bin/env python3
# Decription:
# This program takes user input commands and sends them
# to a seperate program that deals with making the motors
# on the ROSPerch move.
# Adapted Sources:
# Motor command code in auto_test.py is adapted 
# from the UTAP 2020 code at 
# https://github.com/jdicecco/UTAP/
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/dmichael1227/ROSPerch/blob/master/LICENSE

# Imports the necessary libraries and messages
import rospy
import time
from std_msgs.msg import Bool

# Talker node function
def talker():
    pub = rospy.Publisher('motorstuff', Bool, queue_size=10) # Publish to motorstuff topic
    rospy.init_node('talker', anonymous=True) # Initiate the  talker node
    rate = rospy.Rate(10) # Setting the ROS rate to 10hz
    while not rospy.is_shutdown(): # Runs the code only while the roscore is running

        # Prompts the user for an input of either Y or N and assigns that to a variable
        print("Launch mission?")
        command_input = input("Y/N: ") # Query for command
        command_input = command_input.upper() # Converts input to uppercase

         # If the user inputs Y, launch mission
        if command_input == 'Y':
            launch_command = bool(1) # Convert to boolean
            print("Launch_command is : %s" % launch_command)
            rospy.loginfo(launch_command) # Log the entered command
            pub.publish(launch_command) # Publish command to launch_command topic
            rate.sleep() # Sleep (10ms)
            print()

        # If the user inputs N, don't launch mission
        elif command_input == 'N':
            launch_command = bool(0) # Convert to boolean
            print("Launch_command is : %s" % launch_command)
            rospy.loginfo(launch_command) # Log the entered command
            pub.publish(launch_command) # Publish command to launch_command topic
            rate.sleep() # Sleep (10ms)
            print()

        # If the user inputs N, don't launch mission
        else:
            print(command_input, " is an invalid input")
            print("Please try again!")
            print()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass