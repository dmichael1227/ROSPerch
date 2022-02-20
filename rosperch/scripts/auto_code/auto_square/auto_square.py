#!/usr/bin/env python3
# Decription:
# This program makes the ROSPerch move
# in a prepogrammed path, which in this
# case is a square.
# Adapted Sources:
# Motor command code is adapted from the UTAP 2020
# code at https://github.com/jdicecco/UTAP/
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/mxdrew/ROSPerch/blob/master/LICENSE

# Imports the necessary libraries and messages
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

# Drive function
def drive(GRdir,BLdir,t):
    start_time = time.time() # Initialize start time
    curr_time = time.time() # Initialize current time
    print(t)
    while ((curr_time - start_time) < t): # How long to stay in this state/execute command for
        # Sends commands over GPIO to make motors move
        curr_time = time.time()
        if GRdir == True:
            GPIO.output(GR1,GPIO.LOW)
        else:
            GPIO.output(GR1,GPIO.HIGH)
        if BLdir == True:
            GPIO.output(BL1,GPIO.LOW)
        else:
            GPIO.output(BL1,GPIO.HIGH)
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF # Stops motors at end of command
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF # Stops motors at end of command

# Stop function 
def stop_motors():
    pwm.channels[GR1_PWM].duty_cycle = 0x0000 # Stops motors
    pwm.channels[BL1_PWM].duty_cycle = 0x0000 # Stops motors

# Callback function
def callback(data):
    # Prints out logged data and sets variables used in Drive function
    global running_mission
    rospy.loginfo('I heard %s', data.data) # Log what is heard
    print(data.data)
    print(running_mission)
    # If told to run mission, run mission, otherwise, don't
    if data.data == True and running_mission == False:
        running_mission = True
        drive(True,True,3.5)
        drive(True,False,0.8)
        drive(True,True,3.5)
        drive(True,False,1.3)
        drive(True,True,3.2)
        drive(True,False,1.3)
        drive(True,True,3.5)
        drive(True,False,1.3)
        stop_motors()
        running_mission = False
    elif data.data == True and running_mission == True:
        print("Warning: Mission already running")
    else:
        print("Attempting to stop mission...")
        stop_motors()

# Listener node function
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('motorstuff', Bool, callback) # Subscribe to motorstuff topic

    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()