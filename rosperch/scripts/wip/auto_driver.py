#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github/ros/ros_tutorials.com
# Software License Agreement (GPLv3 License)

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
GPIO.setup(GR1,GPIO.OUT) #Green 1
GPIO.setup(GR2,GPIO.OUT) #Green 2
GPIO.setup(BL1,GPIO.OUT) #Blue 1
GPIO.setup(BL2,GPIO.OUT) #Blue 2
GPIO.setup(OR1,GPIO.OUT) #Orange 1
GPIO.setup(BR1,GPIO.OUT) #Brown 1


#status LEDs
GPIO.setup(6,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
running_mission = False

perch_speed_straight = 0.4 #define perch forward speed (m/s)
perch_speed_rightturn = 180 #define perch right turn speed (degrees/s)
perch_speed_leftturn = 180 #define perch left turn speed (degrees/s)
pub = rospy.Publisher('systemstate', Bool, queue_size=10)

#function for driving forwards
def drive(dist):
    global pub
    start_time = time.time() #initialize start time
    curr_time = time.time() #initialize current time
    t = dist/perch_speed_straight
    while (curr_time - start_time) < t
        curr_time = time.time()
        GPIO.output(GR1,GPIO.HIGH) #go forward        
        GPIO.output(BL1,GPIO.HIGH) #go forward
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF #full speed. Adjust these for drift
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF #full speed. Adjust these for drift
        systemready = False
        pub.publish(systemready)
    systemready = True
    pub.publish(systemready)

#function for turning right        
def rightturn(angle):
    global pub
    start_time = time.time() #initialize start time
    curr_time = time.time() #initialize current time
    t = angle/perch_speed_rightturn #how long to stay in this state
    while (curr_time - start_time) < t
        curr_time = time.time()
        GPIO.output(GR1,GPIO.LOW) #reverse        
        GPIO.output(BL1,GPIO.HIGH) #go forward 
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF #full speed.
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF #full speed. 
        systemready = False
        pub.publish(systemready)
    systemready = True
    pub.publish(systemready)
    
#function for turning left        
def leftturn(angle):
    global pub
    start_time = time.time() #initialize start time
    curr_time = time.time() #initialize current time
    t = angle/perch_speed_rightturn #how long to stay in this state
    while (curr_time - start_time) < t
        curr_time = time.time()
        GPIO.output(GR1,GPIO.HIGH) #go forward        
        GPIO.output(BL1,GPIO.LOW) #reverse
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF #full speed.
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF #full speed.
        systemready = False
        pub.publish(systemready)
    systemready = True
    pub.publish(systemready)
    
#function for stopping
def stop_motors():
    pwm.channels[GR1_PWM].duty_cycle = 0x0000
    pwm.channels[BL1_PWM].duty_cycle = 0x0000

def callback(data):
    rospy.loginfo('I heard %s', data.data) # Log what is heard
    if data.command_type == "drive":
        drive(data.command_value)
        print("Driving %s Meters" % data.command_value)
    elif data.command_type == "rightturn":
        rightturn(data.command_value)
        print("Turning Right %s Degrees" % data.command_value)
    elif data.command_type == "leftturn":
        leftturn(data.command_value)
        print("Turning Left %s Degrees" % data.command_value)
    elif data.command_type == "stop":
        stop_motors()
        print("Stopping Motors...")
    else:
        print("Command Not Recognized")

    
def listener():
    # Set up the listener node
    rospy.init_node('listener', anonymous=True) 
    rospy.Subscriber('motorcommands', Commands, callback) # subscribe to motorcommands topic
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
