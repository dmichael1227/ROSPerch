import time
import numpy
import board
import busio
import adafruit_fxos8700
 
 
# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_fxos8700.FXOS8700(i2c)
# Optionally create the sensor with a different accelerometer range (the
# default is 2G, but you can use 4G or 8G values):
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_4G)
# sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)
 
# Main loop will read the acceleration and magnetometer values every second
# and print them out.
accel_x_list = []
accel_y_list = []
accel_z_list = []
n = 0
try:
    while n < 6000: #Time out after 10 minutes
        # Read acceleration
        accel_x, accel_y, accel_z = sensor.accelerometer
        #mag_x, mag_y, mag_z = sensor.magnetometer
        # Print values.
        print(
            "Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})".format(
                accel_x, accel_y, accel_z
            )
        )
        accel_x_list.append(accel_x)
        accel_y_list.append(accel_y)
        accel_z_list.append(accel_z)
        n = n + 1
        # Delay for a tenth of a second.
        time.sleep(0.1)
except:
    accel_x_array = numpy.array(accel_x_list)
    accel_y_array = numpy.array(accel_y_list)
    accel_z_array = numpy.array(accel_z_list)
    
    accel_x_var = numpy.var(accel_x_array)
    accel_y_var = numpy.var(accel_y_array)
    accel_z_var = numpy.var(accel_z_array)
    print('Var: Accel X: %s Accel Y: %s Accel Z: %s',(accel_x_var,accel_y_var,accel_z_var))
print("Execution Ended")    
