#!/usr/bin/env python3
# Decription:
# This program takes user input commands and sends them
# to a seperate program that deals with making the motors
# on the ROSPerch move.
# Adapted Sources:
# Motor command code in auto_driver.py is adapted
# from the UTAP 2020 code at
# https://github.com/jdicecco/UTAP/
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/dmichael1227/ROSPerch/blob/master/LICENSE


# Imports the necessary libraries and messages
import threading
import rospy
import time
from std_msgs.msg import Bool
from rosperch.msg import Commands

# Sets up system state varible
systemready = True # Assume it's ready, if it isn't it'll tell you

# Callback Funtion
def callback(data):
    global systemready
    systemready = data.data
    print("System State %s " % systemready)
    if systemready == True: # If the system is ready...
        pub = rospy.Publisher('motorcommands', Commands, queue_size=10) # Publish to motorcommands topic
        rate = rospy.Rate(10) # Sets the ROS rate to 10hz
        while not rospy.is_shutdown(): # Runs the code only while the roscore is running
            print()
            print("Mission Commands:")
            print("Drive, RightTurn, LeftTurn, Stop")
            print()
            command_input = input("Please Input a Command: ") # Ask user for mission command
            command_input = str(command_input.upper())
            print()

            # What to do if the user sends the Drive Command
            if command_input == "DRIVE":
                command_input = 'DRIVE' # Accounts for differences in input and what the listener is looking for
                mission_parameter = float(input("Distance in Meters: ")) # Asks user for Distance
                rospy.loginfo([command_input," %s" % mission_parameter]) # Log the entered commands
                pub.publish(command_input,mission_parameter) # Publish command_input and mission_parameter
                rate.sleep() # Sleep for 10ms

            # What to do if the user sends the Turn Right Command (accounts for space or no space)
            elif command_input == "RIGHTTURN" or command_input == "RIGHT TURN":
                command_input = 'RIGHTTURN' # Accounts for differences in input and what the listener is looking for
                mission_parameter = float(input("Degrees to Turn: ")) # Asks user for Turn parameters
                rospy.loginfo([command_input," %s" % mission_parameter]) # Log the entered commands
                pub.publish(command_input,mission_parameter)
                rate.sleep() # Sleep for 10ms

            # What to do if the user sends the Turn Left Command (accounts for space or no space)
            # For line by line comments describing functionality, see the DRIVE and RIGHTTURN
            # if/elif statements.
            elif command_input == "LEFTTURN" or command_input == "LEFT TURN":
                command_input = 'LEFTTURN'
                mission_parameter = float(input("Degrees to Turn: "))
                rospy.loginfo([command_input," %s" % mission_parameter])
                pub.publish(command_input,mission_parameter)
                rate.sleep() # Sleep for 10ms

            # What to do if the user sends the Stop Command
            # For line by line comments describing functionality, see the DRIVE and RIGHTTURN
            # if/elif statements.
            elif command_input == "STOP":
                command_input = 'STOP'
                mission_parameter = float(1)
                rospy.loginfo([command_input," %s" % mission_parameter])
                pub.publish(command_input,mission_parameter)
                rate.sleep() # Sleep for 10ms

            # What to do if the user sends an Invalid Command
            else:
                print("You entered an invalid command. Try again.")
                print()
    print("System State %s " % systemready)


# Listener node function 
def listener():
    # Set up the listener node
    rospy.Subscriber('systemstate', Bool, callback) # subscribe to motorcommands topic
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('commander', anonymous=True) #initiate talker node
        listener()
    except rospy.ROSInterruptException:
        pass
