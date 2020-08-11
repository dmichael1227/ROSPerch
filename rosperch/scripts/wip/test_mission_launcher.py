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
        
        # Prompts the user for an input of either Y or N and assigns that to a variable
        print("Launch mission?")
        command_input = input("Y/N: ") # Query for command
        command_input = command.upper()
        
         # If the user inputs Y, launch mission
        if command_input == 'Y':
            launch_command = bool(1) # Convert to boolean
            print("Launch_command is : %s" % launch_command)
            rospy.loginfo(launch_command) # Log the entered command
            pub.publish(launch_command) # Publish command to ledstuff topic
            rate.sleep() # Sleep (10ms)
            print()

        # If the user inputs N, don't launch mission
        elif command_input == 'N':
            launch_command = bool(0) # Convert to boolean
            print("Launch_command is : %s" % launch_command)
            rospy.loginfo(launch_command) # Log the entered command
            pub.publish(launch_command) # Publish command to ledstuff topic
            rate.sleep() # Sleep (10ms)
            print()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
