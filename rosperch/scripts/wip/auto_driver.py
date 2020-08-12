#!/usr/bin/env python3
# Decription:
# This program takes user input commands and sends them
# to a seperate program that deals with making the motors
# on the ROSPerch move.
# Adapted Sources:
# Motor command code is adapted from the UTAP 2020
# code at https://github.com/jdicecco/UTAP/
# License:
# Software License Agreement (GPLv3 License)
# Find the full agreement at https://github.com/amichael1227/ROSPerch/blob/master/LICENSE


# Imports the necessary libraries and messages
import rospy
import time
import RPi.GPIO as GPIO
import adafruit_pca9685
import board
from rosperch.msg import Commands
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
def stop_motors():
    pwm.channels[GR1_PWM].duty_cycle = 0x0000
    pwm.channels[BL1_PWM].duty_cycle = 0x0000
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

# Sets up variables used later on
running_mission = False # Defines variable saying if mission is currently running
perch_speed_straight = 0.4 # Defines ROSPerch forward speed (m/s)
perch_speed_rightturn = 180 # DefineS ROSPerch right turn speed (degrees/s)
perch_speed_leftturn = 180 # Define ROSPerch left turn speed (degrees/s)

# Publishes the system state
pub = rospy.Publisher('systemstate', Bool, queue_size=10)

# Function for driving forwards
def drive(dist):
    global pub # Grabs a global variable
    start_time = time.time() # Initialize start time
    curr_time = time.time() # Initialize current time
    t = dist/perch_speed_straight # How long to stay in this state/execute command for
    while (curr_time - start_time) < t:
        curr_time = time.time()
        GPIO.output(GR1,GPIO.HIGH) # Go forward        
        GPIO.output(BL1,GPIO.HIGH) # Go forward
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift
        systemready = False # Sets system state
        pub.publish(systemready)
    systemready = True # Sets system state
    pub.publish(systemready) # Publishes system state
    pwm.channels[GR1_PWM].duty_cycle = 0x0000 # Stops motors once command is over
    pwm.channels[BL1_PWM].duty_cycle = 0x0000 # Stops motors once command is over

# Function for turning right   
# Comments that are repeated throughout the movement
# functions have been removed for brevity.      
def rightturn(angle):
    global pub # Grabs a global variable
    start_time = time.time()
    curr_time = time.time()
    t = angle/perch_speed_rightturn
    while (curr_time - start_time) < t:
        curr_time = time.time()
        GPIO.output(GR1,GPIO.LOW) # Reverse        
        GPIO.output(BL1,GPIO.HIGH) # Go forward 
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift 
        systemready = False
        pub.publish(systemready)
    systemready = True
    pub.publish(systemready)
    pwm.channels[GR1_PWM].duty_cycle = 0x0000
    pwm.channels[BL1_PWM].duty_cycle = 0x0000 

# Function for turning left        
def leftturn(angle):
    global pub
    start_time = time.time()
    curr_time = time.time()
    t = angle/perch_speed_rightturn
    while (curr_time - start_time) < t:
        curr_time = time.time()
        GPIO.output(GR1,GPIO.HIGH) #go forward        
        GPIO.output(BL1,GPIO.LOW) #reverse
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF # Full speed. Be sure to adjust these for drift
        systemready = False
        pub.publish(systemready)
    systemready = True
    pub.publish(systemready)
    pwm.channels[GR1_PWM].duty_cycle = 0x0000 # Stop
    pwm.channels[BL1_PWM].duty_cycle = 0x0000 # Stop
    
# Function for stopping
def stop_motors():
    pwm.channels[GR1_PWM].duty_cycle = 0x0000 # Stop
    pwm.channels[BL1_PWM].duty_cycle = 0x0000 # Stop
    systemready = True
    pub.publish(systemready)
    
# Function to be run when an unrecognized command is received
def bad_command():
    global pub
    systemready = True
    pub.publish(systemready)

# Callback function
def callback(data):
    rospy.loginfo('I heard %s %s', data.command_type,data.command_value) # Log what is heard
    # Prints out the logged data
    if data.command_type == "DRIVE":
        drive(data.command_value)
        print("Driving %s Meters" % data.command_value)
    elif data.command_type == "RIGHTTURN":
        rightturn(data.command_value)
        print("Turning Right %s Degrees" % data.command_value)
    elif data.command_type == "LEFTTURN":
        leftturn(data.command_value)
        print("Turning Left %s Degrees" % data.command_value)
    elif data.command_type == "STOP":
        stop_motors()
        print("Stopping Motors...")
    else:
        bad_command()
        print("Command Not Recognized")

# Listener node function
def listener():
    # Set up the listener node
    #rospy.init_node('listener', anonymous=True) 
    rospy.Subscriber('motorcommands', Commands, callback) # Subscribe to motorcommands topic
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

#def handshake():
#    print("Attempting Shake")
#    rospy.init_node('listener', anonymous=True)
#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        handshaker = rospy.Publisher("handshake",Bool,queue_size=10)
#        handshaker.publish(True)
#        rate.sleep()

if __name__ == '__main__':
#    handshake()
    rospy.init_node('listener',anonymous=True)
    systemready = True
    pub.publish(systemready)
    listener()