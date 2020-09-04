# Simple demo of the FXOS8700 accelerometer and magnetometer.
# Will print the acceleration and magnetometer values every second.
import time
 
import board
import busio
 
import adafruit_fxos8700
import adafruit_fxas21002c
import numpy
 
 
# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_fxos8700.FXOS8700(i2c)
sensor2 = adafruit_fxas21002c.FXAS21002C(i2c)
# Optionally create the sensor with a different accelerometer range (the
# default is 2G, but you can use 4G or 8G values):
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_4G)
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)
 
# Main loop will read the acceleration and magnetometer values every second
# and print them out.
accel_x_list = []
accel_y_list = []
accel_z_list = []
gyro_x_list = []
gyro_y_list = []
gyro_z_list = []
n = 0
#Record data for a minute
while n < 600:
    # Read acceleration & magnetometer.
    accel_x, accel_y, accel_z = sensor.accelerometer
    #mag_x, mag_y, mag_z = sensor.magnetometer
    # Print values.
    print(
        "Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
            accel_x, accel_y, accel_z
        )
    )
    # Read gyroscope.
    gyro_x, gyro_y, gyro_z = sensor2.gyroscope
    # Print values.
    print(
        "Gyroscope (radians/s): ({0:0.3f},  {1:0.3f},  {2:0.3f})".format(
            gyro_x, gyro_y, gyro_z
        )
    )
    accel_x_list.append(accel_x)
    accel_y_list.append(accel_y)
    accel_z_list.append(accel_z)
    gyro_x_list.append(gyro_x)
    gyro_y_list.append(gyro_y)
    gyro_z_list.append(gyro_z)
    n = n+1
    # Delay for a tenth of a second.
    time.sleep(0.1)
#Convert to numpy array    
accel_x_array = numpy.array(accel_x_list)
accel_y_array = numpy.array(accel_y_list)
accel_z_array = numpy.array(accel_z_list)

gyro_x_array = numpy.array(gyro_x_list)
gyro_y_array = numpy.array(gyro_y_list)
gyro_z_array = numpy.array(gyro_z_list)
#Take Variance
accel_x_var = numpy.var(accel_x_array)
accel_y_var = numpy.var(accel_y_array)
accel_z_var = numpy.var(accel_z_array)

gyro_x_var = numpy.var(gyro_x_array)
gyro_y_var = numpy.var(gyro_y_array)
gyro_z_var = numpy.var(gyro_z_array)
#Print
print('Var: Accel X: %s Accel Y: %s Accel Z: %s',(accel_x_var,accel_y_var,accel_z_var))
print('Var: Gyro X: %s Gyro Y: %s Gyro Z: %s',(gyro_x_var,gyro_y_var,gyro_z_var))
