#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# Software License Agreement (BSD License)


import rospy
import subprocess
from std_msgs.msg import Bool

FILE_PATH = '/home/ubuntu/catkin_ws/src/rosperch/scripts/simple_motor_test/motor_code.py'

def callback(data):
    rospy.loginfo('I heard %s', data.data) #log what is heard
    subprocess.call(['sudo','python3', FILE_PATH,'%s' % data.data]) # call led_blinker.py when receiving a message

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) 

    rospy.Subscriber('motorstuff', Bool, callback) # subscribe to ledstuff topic
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

