#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# Software License Agreement (BSD License)

import threading
import rospy
import time
from std_msgs.msg import Bool
from rosperch.msg import Commands


systemready = True
def talker():
    global systemready
    pub = rospy.Publisher('motorcommands', Commands, queue_size=10) #publish to ledstuff topic
    rospy.init_node('talker', anonymous=True) #initiate talker node
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber('motorcommands', Commands, callback=None) # subscribe to motorcommands topic
        command_input = input("Mission Command (drive, rightturn,leftturn,stop): ") #query for command and convert to float
        mission_parameter = float(input("Mission Parameter (dist,degrees): "))
        rospy.loginfo([launch_command," %s" % mission_parameter]) #log the entered commands
        if systemready:
            pub.publish(launch_command) #publish command to ledstuff topic
        else:
            print("System Not Ready!")
        rate.sleep() #sleep (10ms)

def callback(data):
    global systemready
    systemready = data.data
    print("System State %s " % systemready)
    
def listener():
    global systemready
    # Set up the listener node
    rospy.init_node('listener', anonymous=True) 
    rospy.Subscriber('systemstate', Commands, callback) # subscribe to motorcommands topic
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()
    
if __name__ == '__main__':
    try:
        t = threading.Thread(target=listener,args=None,daemon=True).start
        talker()
    except rospy.ROSInterruptException:
        pass
