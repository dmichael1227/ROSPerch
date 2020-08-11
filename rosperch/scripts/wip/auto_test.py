#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# And the UTAP 2020 Code at https://github.com/jdicecco/UTAP/blob/master/UTAP_2020.py
# Software License Agreement (BSD License)

import rospy
import time
import RPi.GPIO as GPIO
import sys
import getopt
import adafruit_pca9685
import board
from std_msgs.msg import Bool
# Set up the PWM Board
i2c_pwm = board.I2C()
pwm = adafruit_pca9685.PCA9685(i2c_pwm)
pwm.frequency = 1600

#### CONFIGURE THE RPI TO INTERFACE WITH CONTROL BOARD ####

# Make it easier to remember which pins control which motors
GR1 = 19
GR2 = 21
BL1 = 13
BL2 = 26
OR1 = 20
BR1 = 27

# Do the same for the corresponding PWM signals
GR1_PWM = 1
GR2_PWM = 5
BL1_PWM = 3
BL2_PWM = 6
OR1_PWM = 0
BR1_PWM = 2

# Use the numbering scheme for the Broadcom chip, not the RPi pin numbers
GPIO.setmode(GPIO.BCM)

# Turn off warnings about pins being already configured
GPIO.setwarnings(False)

# Setup pins to control direction on the motor driver chip (MAXIM's MAX14870)
GPIO.setup(GR1,GPIO.OUT) # Green 1
GPIO.setup(GR2,GPIO.OUT) # Green 2
GPIO.setup(BL1,GPIO.OUT) # Blue 1
GPIO.setup(BL2,GPIO.OUT) # Blue 2
GPIO.setup(OR1,GPIO.OUT) # Orange 1
GPIO.setup(BR1,GPIO.OUT) # Brown 1


# Status LEDs
GPIO.setup(6,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
running_mission = False

def drive(GRdir,BLdir,t):
    start_time = time.time
    curr_time = time.time
    print(t)
    while ((curr_time - start_time) < t):
        curr_time = time.time
        if GRdir == True:
            GPIO.output(GR1,GPIO.HIGH)
        else:
            GPIO.output(GR1,GPIO.LOW)
        if BLdir == True:
            GPIO.output(BL1,GPIO.HIGH)
        else:
            GPIO.output(BL1,GPIO.LOW)
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF

def stop_motors():
    pwm.channels[GR1_PWM].duty_cycle = 0x0000
    pwm.channels[BL1_PWM].duty_cycle = 0x0000

def callback(data):
    global running_mission
    rospy.loginfo('I heard %s', data.data) # Log what is heard
    if data.data == True & running_mission == False:
        running_mission = True
        drive(True,True,3.77)
        drive(True,False,0.5)
        drive(True,True,1.0)
        stop_motors()
        running_mission = False
    elif data.data == True & running_mission == True:
        print("Warning: Mission already running")
    else:
        print("Attempting to stop mission...")
        stop_motors()

def listener():
    # Set up the listener node
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('motorstuff', Bool, callback) # Subscribe to ledstuff topic

    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
