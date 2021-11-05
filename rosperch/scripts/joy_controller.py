#!/usr/bin/env python3
# Decription:
# This program takes user input commands from the 
# external controller, and moves the ROSPerch as such.
# Adapted Sources:
# Motor command code is adapted from the UTAP 2020
# code at https://github.com/jdicecco/UTAP/
# Joystick control code is based on code released 
# by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/dmichael1227/ROSPerch/blob/master/LICENSE

# Imports the necessary libraries
import time
import math
import board
import busio
import adafruit_pca9685
import subprocess
import os, sys, struct, array
from fcntl import ioctl
import RPi.GPIO as GPIO

# I2C address for the PWM driver board retrieved automatically
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


# Based on code released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

# Find the joystick device(s)
print('Available devices:')

# Need to check to make sure a joystick has been connected before we proceed
# if not, we'll just wait here until someone connects a joystick.

# This is usually called a flag and is used to check a condition
# when the desired condition is met, we change the value of the flag.
joy_not_found = 1

while joy_not_found:
    for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('  /dev/input/%s' % (fn))
            joy_not_found = 0


# We'll store the states of the axes and buttons
axis_states = {}
button_states = {}

# These constants were borrowed and modified from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'rx',
    0x03 : 'x2',
    0x04 : 'y2',
    0x05 : 'ry',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
}

button_names = {
    0x130 : 'a',
    0x131 : 'b',
    0x133 : 'x',
    0x134 : 'y',
    0x136 : 'LB',
    0x137 : 'RB',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print(('%d axes found: %s' % (num_axes, ', '.join(axis_map))))
print(('%d buttons found: %s' % (num_buttons, ', '.join(button_map))))

# Declare variables for use later
# These will be the values from the right joystick
intValx2 = 0
intValy2 = 0

# These will be the values from the left joystick
intValx = 0
intValy = 0

# These will be the values from the two triggers in the front of the joystick
intValrx = 0
intValry = 0

# A TRY-CATCH in programming allows a program to fail gracefully if the "try" portion
# cannot be executed.
try:

    # Main event loop
    while True:

        # Joystick code based on release by rdb under the Unlicense (unlicense.org)
        # Based on information from:
        # https://www.kernel.org/doc/Documentation/input/joystick-api.txt

        evbuf = jsdev.read(8)
        if evbuf:
            tyme, value, type, number = struct.unpack('IhBB', evbuf)

            # Use for debugging
            #if type & 0x80:
                #print("(initial)",end=""),

            if type & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    # Use "PRINT" for debugging - comment out to speed program execution
                    if value:
                        print("%s pressed" % (button))
                    else:
                        print("%s released" % (button))
                    if button == "y":
                        GPIO.output(6,GPIO.HIGH)#turn on other LED
                    else:
                        GPIO.output(6,GPIO.LOW)#otherwise turn it off - should turn off when any other button is pushed
                    if button == "x":
                        GPIO.output(GR2,GPIO.HIGH)
                        GPIO.output(BL2,GPIO.HIGH)
                        pwm.channels[GR2_PWM].duty_cycle = 0xFFFF
                        pwm.channels[BL2_PWM].duty_cycle = 0xFFFF
                        GPIO.output(16,GPIO.HIGH)
                    else:
                        GPIO.output(GR2,GPIO.LOW)
                        GPIO.output(BL2,GPIO.LOW)
                        pwm.channels[GR2_PWM].duty_cycle = 0
                        pwm.channels[BL2_PWM].duty_cycle = 0
                        GPIO.output(16,GPIO.LOW)
            if type & 0x02:
                axis = axis_map[number]
                #right joystick fwd/rev
                if axis=="y2":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValy2 = int(fvalue)*2+1
                    # Use "PRINT" for debugging, comment out to speed program
                    print("%d" % (intValy2))
                # Right joystick left/right
                if axis=="x2":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValx2 = int(fvalue)*2+1
                # Left joystick fwd/rev
                if axis=="y":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValy = int(fvalue)*2+1
                # Left joystick left/right
                if axis=="x":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValx = int(fvalue)*2+1
                # Front right trigger fwd (vehicle ascend)
                if axis=="ry":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValry = int(fvalue)*2+1
                # Front left trigger rev (vehicle descend)
                if axis=="rx":
                    fvalue = value
                    axis_states[axis] = fvalue
                    intValrx = int(fvalue)*2+1

                # There's a nice tutorial for single joysick control at http://home.kendra.com/mauser/Joystick.html
                if intValy2<-100:

                    GPIO.output(GR1,GPIO.LOW) # Direction pin
                    pwm.channels[GR1_PWM].duty_cycle = abs(intValy2)

                elif intValy2>100:

                    GPIO.output(GR1,GPIO.HIGH) # Direction pin
                    pwm.channels[GR1_PWM].duty_cycle = (intValy2)

                else:
                    pwm.channels[GR1_PWM].duty_cycle = 0

                if intValy>100:
                    GPIO.output(BL1,GPIO.HIGH)#direction pin
                    pwm.channels[BL1_PWM].duty_cycle = (intValy)

                elif intValy<-100:
                    GPIO.output(BL1,GPIO.LOW)#direction pin
                    pwm.channels[BL1_PWM].duty_cycle = abs(intValy)

                else:
                    pwm.channels[BL1_PWM].duty_cycle = 0

                if intValrx>100:
                    GPIO.output(OR1,GPIO.LOW)#direction pin
                    GPIO.output(BR1,GPIO.LOW)#direction pin
                    pwm.channels[OR1_PWM].duty_cycle = abs(intValrx)
                    pwm.channels[BR1_PWM].duty_cycle = abs(intValrx)

                elif intValry>100:
                    GPIO.output(OR1,GPIO.HIGH)#direction pin
                    GPIO.output(BR1,GPIO.HIGH)#direction pin
                    pwm.channels[OR1_PWM].duty_cycle = abs(intValry)
                    pwm.channels[BR1_PWM].duty_cycle = abs(intValry)

                else:
                    pwm.channels[OR1_PWM].duty_cycle = 0
                    pwm.channels[BR1_PWM].duty_cycle = 0

except (KeyboardInterrupt,SystemExit):
    pwm.channels[OR1_PWM].duty_cycle = 0
    pwm.channels[BR1_PWM].duty_cycle = 0
    pwm.channels[BL1_PWM].duty_cycle = 0
    pwm.channels[GR1_PWM].duty_cycle = 0
    GPIO.output(6,GPIO.LOW)
    GPIO.output(16,GPIO.LOW)
    GPIO.cleanup()