#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# And the UTAP 2020 Code at https://github.com/jdicecco/UTAP/blob/master/UTAP_2020.py
# Software License Agreement (BSD License)

import rospy
import time
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('motorstuff', Bool, queue_size=10) # Publish to motorstuff topic
    rospy.init_node('talker', anonymous=True) # Initiate the  talker node
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
