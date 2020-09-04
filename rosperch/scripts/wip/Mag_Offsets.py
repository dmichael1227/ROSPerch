# Simple demo of the FXOS8700 accelerometer and magnetometer.
# Will print the acceleration and magnetometer values every second.
import time

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
max_mag_x = 0
max_mag_y = 0
max_mag_z = 0
min_mag_x = 0
min_mag_y = 0
min_mag_z = 0
while True:
    # Read acceleration & magnetometer.
    #accel_x, accel_y, accel_z = sensor.accelerometer
    mag_x, mag_y, mag_z = sensor.magnetometer
    #Update max and min vals
    if mag_x > max_mag_x:
        max_mag_x = mag_x
    if mag_x < min_mag_x:
        min_mag_x = mag_x
    if mag_y > max_mag_y:
        max_mag_y = mag_y
    if mag_y < min_mag_y:
        min_mag_y = mag_y
    if mag_z > max_mag_z:
        max_mag_z = mag_z
    if mag_z < min_mag_z:
        min_mag_z = mag_z
    #Calculate hard iron offset    
    mag_x_offset = (max_mag_x + min_mag_x)/2 
    mag_y_offset = (max_mag_y + min_mag_y)/2 
    mag_z_offset = (max_mag_z + min_mag_z)/2
    
    print("Offsets- X: %s Y: %s Z: %s ",(mag_x_offset,mag_y_offset,mag_z_offset))
    # Delay for a tenth of a second.
    
    time.sleep(0.1)
