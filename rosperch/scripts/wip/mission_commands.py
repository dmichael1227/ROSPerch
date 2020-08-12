#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# Software License Agreement (GPLv3 License)

import threading
import rospy
import time
from std_msgs.msg import Bool
from rosperch.msg import Commands

systemready = True #Assume it's ready, if it isn't it'll tell you

def callback(data):
    global systemready
    systemready = data.data
    print("System State %s " % systemready)
    if systemready == True:
        pub = rospy.Publisher('motorcommands',Commands,queue_size=10)
        command_input = input("Mission Command (drive, rightturn, leftturn, stop): ")
        mission_parameter = float(input("Mission Parameter (dist, degrees): "))
        rospy.loginfo([command_input," %s" % mission_parameter])
        pub.publish(command_input,mission_parameter)
    print("System State %s " % systemready)
    
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
