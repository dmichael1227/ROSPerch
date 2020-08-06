#!/usr/bin/env python3
# Based on the simple publisher/subscriber tutorial on the ROS tutorial page:
# https://github.com/ros/ros_tutorials
# Also based on the UTAP 2020 code available at:
# https://github.com/jdicecco/UTAP
# Software License Agreement (BSD License)

import RPi.GPIO as GPIO
import sys
import getopt
from time import sleep
import adafruit_pca9685
import board


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
GPIO.setup(GR1,GPIO.OUT) #Green 1
GPIO.setup(GR2,GPIO.OUT) #Green 2
GPIO.setup(BL1,GPIO.OUT) #Blue 1
GPIO.setup(BL2,GPIO.OUT) #Blue 2
GPIO.setup(OR1,GPIO.OUT) #Orange 1
GPIO.setup(BR1,GPIO.OUT) #Brown 1


#status LEDs
GPIO.setup(6,GPIO.OUT)#
GPIO.setup(16,GPIO.OUT)#

try:
    # Turns on motors and LEDs if input is true
    if sys.argv[1] == "True":
        GPIO.output(GR1,GPIO.HIGH)
        GPIO.output(BL1,GPIO.HIGH)
        pwm.channels[GR1_PWM].duty_cycle = 0xFFFF
        pwm.channels[BL1_PWM].duty_cycle = 0xFFFF
        GPIO.output(16,GPIO.HIGH)
        GPIO.output(6,GPIO.HIGH)

    # Turns off motors and LEDs if input is false
    elif sys.argv[1] == "False":
        GPIO.output(GR1,GPIO.HIGH)
        GPIO.output(BL1,GPIO.HIGH)
        pwm.channels[GR1_PWM].duty_cycle = 0
        pwm.channels[BL1_PWM].duty_cycle = 0
        GPIO.output(16,GPIO.LOW)
        GPIO.output(6,GPIO.LOW)

    # If not true or fase, let user know there's an error
    else:
        print("ERROR: EXPECTING BOOL VALUE")

# Exit gracefully and cleanup
except (KeyboardInterrupt,SystemExit):
    GPIO.cleanup()
