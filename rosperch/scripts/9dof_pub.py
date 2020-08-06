import time
import math
import board
import busio
import rospy

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
def talker():
    pub = rospy.Publisher('orientation', Orientation, queue_size=10) #publish to ledstuff topic
    rospy.init_node('talker', anonymous=True) #initiate talker node
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        try:             
            #Previous code for reading differnt IMU sensor
            #mag_x, mag_y, mag_z = mag_sensor.magnetic
            #accel_x, accel_y, accel_z = accel_sensor.acceleration
            
            #Read IMU
            mag_x, mag_y, mag_z = mag_accel_sensor.magnetometer
            accel_x, accel_y, accel_z = mag_accel_sensor.accelerometer
            gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope
            
            
            #mag calibration offsets for a SPECIFIC device - yours will be different!!
            #X: -46.20, Y:  -83.05, Z: -107.25
            #X: -38.30, Y:  -67.60, Z: -100.35
            
            mag_cal_x = -38.3
            mag_cal_y = -67.6
            mag_cal_z = -100.35
            
            mag_x = mag_x-mag_cal_x
            mag_y = mag_y-mag_cal_y
            mag_z = mag_z-mag_cal_z
            
            yaw = math.atan2(mag_y,mag_x)
            
            #We use print statements for debugging - comment out to spee execution
            #print("mag: {},{},{}".format(mag_x, mag_y, mag_z))
            #print("accel: {}".format(mag_accel_sensor.accelerometer))
        
            pitch = math.atan2(accel_x,math.sqrt(accel_y**2+accel_z**2))
            roll = math.atan2(accel_y,math.sqrt(accel_x**2+accel_z**2))
  
            if yaw < 0:
                yaw=yaw+2*math.pi
                
            #tilt compensation
            x_h = mag_x*math.cos(pitch) + mag_z*math.sin(pitch)
            y_h = mag_x*math.sin(roll)*math.sin(pitch)+mag_y*math.cos(roll)-mag_z*math.sin(roll)*math.cos(pitch)
            
            tilt_yaw = math.atan2(y_h,x_h)
            
            if tilt_yaw < 0:
                tilt_yaw=tilt_yaw+2*math.pi
            #convert radians to degrees
            rollDeg = roll*57.2958            
            pitchDeg = pitch*57.2958
            yawDeg = yaw*57.2958
            yawTilt = tilt_yaw*57.2958
            rospy.loginfo("Roll: %s Pitch: %s Yaw: %s Tilt Yaw: %s ",rollDeg, pitchDeg,
                          yawDeg, yawTilt) #log magnetometer stuff
            rospy.loginfo("Accel X: %s Accel Y: %s Accel Z: %s", accel_x, accel_y, accel_z)
            pub.publish(rollDeg, pitchDeg, yawDeg, yawTilt, accel_x, accel_y, accel_z) #publish sensor data 
            rate.sleep() #sleep (10ms)

        except:

            subprocess.call(['i2cdetect', '-y', '1'])
            rate.sleep()
            continue
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

