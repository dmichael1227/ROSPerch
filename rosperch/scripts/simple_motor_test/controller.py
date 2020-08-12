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
import time
from std_msgs.msg import Bool

# Talker node function
def talker():
    pub = rospy.Publisher('motorstuff', Bool, queue_size=10) # Publish to motorstuff topic
    rospy.init_node('talker', anonymous=True) # Initiate the talker node
    rate = rospy.Rate(10) # Setting the rate to 10hz
    while not rospy.is_shutdown():

        # Prompts the user for an input of either 1 or 0 and assigns that to a variable
        print("Please enter a 1 or a 0!")
        command_input = input("MOTOR COMMAND: ")

        # If the user inputs 0, send off a message to kill the motors and LEDs
        if command_input == '0':
            motor_command = bool(float(command_input)) # Convert inut to float then boolean
            print("motor_command is : %s" % motor_command) # Prompt the user
            rospy.loginfo(motor_command) # Log the entered command
            pub.publish(motor_command) # Publish command to motorstuff topic
            rate.sleep() # Sleep for 10 ms
            print()

        # If the user inputs 1, send off a message to start the motors and LEDs
        elif command_input == '1':
            motor_command = bool(float(command_input))
            print("motor_command is : %s" % motor_command)
            rospy.loginfo(motor_command)
            pub.publish(motor_command)
            rate.sleep()
            print()

        # If the user inputs something other than 0 or 1, prompt them for real input
        else:
            print(command_input, "is an invalid input!")
            print("Please try again!")
            print()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass