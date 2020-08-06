#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# Software License Agreement (BSD License)

import rospy
import subprocess
from std_msgs.msg import Bool

# Defines the path of the code that moves the motors
FILE_PATH = '/home/ubuntu/catkin_ws/src/rosperch/scripts/simple_motor_test/motor_code.py'

def callback(data):
    rospy.loginfo('I heard %s', data.data) # Log what is heard
    subprocess.call(['sudo','python3', FILE_PATH,'%s' % data.data]) # Call motor_code.py when receiving a message

def listener():
    # Set up the listener node
    rospy.init_node('listener', anonymous=True) 
    rospy.Subscriber('motorstuff', Bool, callback) # subscribe to ledstuff topic

    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

