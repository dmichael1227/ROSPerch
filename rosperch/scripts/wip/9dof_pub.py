#!/usr/bin/env python3
# Decription:
# This program gets and prints out values
# from the NXP Absolute Orientation Breakout.
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/dmichael1227/ROSPerch/blob/master/LICENSE

import time
import math
import board
import busio
import rospy
from tf.transformations import *
#We tried several IMU sensors - may go back to this one
#import adafruit_lsm303dlh_mag
#import adafruit_lsm303_accel
#import adafruit_l3gd20
import adafruit_fxos8700
import adafruit_fxas21002c
#Temp, Humidity, Pressure Sensor
#import adafruit_bme280
#Error handling
import subprocess
from rosperch.msg import Orientation
i2c_nxp = board.I2C()
mag_accel_sensor = adafruit_fxos8700.FXOS8700(i2c_nxp)
gyro_sensor = adafruit_fxas21002c.FXAS21002C(i2c_nxp)
last_time = time.time()
distance = 0
distance_cor = 0
curr_vel = 0
curr_vel_cor = 0
count = 0
def talker():
    global last_time
    global distance
    global distance_cor
    global curr_vel
    global curr_vel_cor
    global count
    pub = rospy.Publisher('orientation', Orientation, queue_size=10) #publish to ledstuff topic
    rospy.init_node('talker', anonymous=True) #initiate talker node
    rate = rospy.Rate(200) # 10hz
    while not rospy.is_shutdown():
        try:
            count = count+1
        # Previous code for reading differnt IMU sensor
        #mag_x, mag_y, mag_z = mag_sensor.magnetic
        #accel_x, accel_y, accel_z = accel_sensor.acceleration

            # Read IMU
            mag_x, mag_y, mag_z = mag_accel_sensor.magnetometer
            accel_x, accel_y, accel_z = mag_accel_sensor.accelerometer
            gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope

            # Calculate difference in system time between now and last loop (hopefully ~10ms)
            time_now = time.time()
            time_diff= time_now -  last_time
            last_time = time.time() # Update previous time point (hope that subtraction doesn't take too long!)
            #rospy.loginfo("Time diff: %s" % time_diff)
            distance = distance + 1/2*time_diff*time_diff*accel_x
            #print("Dist: %s" % distance)
            # mag calibration offsets for a SPECIFIC device - yours will be different!!
            #X: -46.20, Y:  -83.05, Z: -107.25
            #X: -38.30, Y:  -67.60, Z: -100.35

            mag_cal_x = -38.3
            mag_cal_y = -67.6
            mag_cal_z = -100.35

            mag_x = mag_x-mag_cal_x
            mag_y = mag_y-mag_cal_y
            mag_z = mag_z-mag_cal_z

            yaw = math.atan2(mag_y,mag_x)

            # We use print statements for debugging - comment out to spee execution
            #print("mag: {},{},{}".format(mag_x, mag_y, mag_z))
            #print("accel: {}".format(mag_accel_sensor.accelerometer))

            pitch = math.atan2(accel_x,math.sqrt(accel_y**2+accel_z**2))
            roll = math.atan2(accel_y,math.sqrt(accel_x**2+accel_z**2))

            if yaw < 0:
                yaw=yaw+2*math.pi

            # Tilt compensation
            x_h = mag_x*math.cos(pitch) + mag_z*math.sin(pitch)
            y_h = mag_x*math.sin(roll)*math.sin(pitch)+mag_y*math.cos(roll)-mag_z*math.sin(roll)*math.cos(pitch)

            tilt_yaw = math.atan2(y_h,x_h)

            if tilt_yaw < 0:
                tilt_yaw=tilt_yaw+2*math.pi

            #Convert radians to degrees
            rollDeg = roll*57.2958
            pitchDeg = pitch*57.2958
            yawDeg = yaw*57.2958
            yawTilt = tilt_yaw*57.2958
            #rospy.loginfo("Roll: %s Pitch: %s Yaw: %s Tilt Yaw: %s ",rollDeg, pitchDeg,
            #              yawDeg, yawTilt) #log magnetometer stuff
            #rospy.loginfo("Accel X: %s Accel Y: %s Accel Z: %s", accel_x, accel_y, accel_z)
            pub.publish(rollDeg, pitchDeg, yawDeg, yawTilt, accel_x, accel_y, accel_z) # Publish sensor data
            #q = quaternion_from_euler(pitch,roll,yaw,'rxyz')
            #rospy.loginfo("Quat: %s" % q)
            #g = [0.0, 0.0, 0.0]
            #g[0] = 2* (q[1]*q[0] - q[3]*q[2])
            #g[1] = 2* (q[3]*q[1] + q[2]*q[0])
            #g[2] = q[3]*q[3] - q[1]*q[1] - q[2]*q[2] + q[0]*q[0]
            #acc_cor_x=accel_x - g[0]
            #acc_cor_y=accel_y - g[1]
            #acc_cor_z=accel_z - g[2]
            #rospy.loginfo("X: %s Y: %s Z: %s",acc_cor_x,acc_cor_y,acc_cor_z)
            acc_cor_x=accel_x - 9.81*math.sin(pitch)
            acc_cor_y = accel_y - 9.81*math.sin(roll)
            #rospy.loginfo("X: %s Y: %s",acc_cor_x,acc_cor_y)
            distance_cor = distance_cor + time_diff*time_diff*acc_cor_x*0.5
            if count >= 100: # Log twice per second
                rospy.loginfo("Distance X: %s" % distance_cor)
                count=0
            rate.sleep() # Sleep (10ms)

        except:

            subprocess.call(['i2cdetect', '-y', '1'])
            rate.sleep()
            continue
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
