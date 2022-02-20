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
# Software License Agreement (GPLv3 License)
# Find the full agreement at https://github.com/mxdrew/ROSPerch/blob/master/LICENSE


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
            print("Mission Commands:")
            print("Drive, Right Turn, Left Turn, Stop")   
            command_input = input("Please Input a Command: ") # Ask user for mission command
            print()

            # What to do if the user sends the Drive Command
            if command_input.upper() == 'DRIVE':
                command_input = 'DRIVE' # Accounts for differences in input and what the listener is looking for
                mission_parameter = float(input("Distance in Meters: ")) # Asks user for Distance
                rospy.loginfo([command_input," %s" % mission_parameter]) # Log the entered commands
                pub.publish(command_input,mission_parameter) # Publish command_input and mission_parameter
                rate.sleep() # Sleep for 10ms

            # What to do if the user sends the Turn Right Command (accounts for space or no space)
            elif command_input.upper() == 'RIGHTTURN' or 'RIGHT TURN':
                command_input = 'RIGHTTURN' # Accounts for differences in input and what the listener is looking for
                mission_parameter = float(input("Degrees to Turn: ")) # Asks user for Turn parameters
                rospy.loginfo([command_input," %s" % mission_parameter]) # Log the entered commands
                pub.publish(command_input,mission_parameter)
                rate.sleep() # Sleep for 10ms            

            # What to do if the user sends the Turn Left Command (accounts for space or no space)
            # For line by line comments describing functionality, see the DRIVE and RIGHTTURN
            # if/elif statements.
            elif command_input.upper() == 'LEFTTURN' or 'LEFT TURN':
                command_input = 'LEFTTURN'
                mission_parameter = float(input("Degrees to Turn: "))
                rospy.loginfo([command_input," %s" % mission_parameter]) 
                pub.publish(command_input,mission_parameter)
                rate.sleep() # Sleep for 10ms       
                
            # What to do if the user sends the Stop Command
            # For line by line comments describing functionality, see the DRIVE and RIGHTTURN
            # if/elif statements.
            elif command_input.upper() == 'STOP':
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

def handshake_publisher(arg1):
    rate = rospy.Rate(1) # Sets the ROS rate to 1hz
    while not rospy.is_shutdown(): # Runs the code only while the roscore is running
        pub = rospy.Publisher('command_to_driver', Bool, queue_size=10) # Publish to command_to_driver topic
        pub.publish(True)
        rate.sleep()

# Listener node function
def listener():
    # Set up the listener node
    rospy.Subscriber('systemstate', Bool, callback) # subscribe to systemstate topic
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('commander', anonymous=True) #initiate talker node
        t = threading.Thread(target=handshake_publisher,args=(1,),daemon=True).start()
        print("Started Handshake Publisher Thread")
        listener()
    except rospy.ROSInterruptException:
        pass

