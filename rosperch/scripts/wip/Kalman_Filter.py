import time
import math
import board
import busio
 
import adafruit_fxos8700
import adafruit_fxas21002c
import numpy
  
# Initialize I2C bus and device.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_fxos8700.FXOS8700(i2c)
sensor2 = adafruit_fxas21002c.FXAS21002C(i2c)

# Initialize hard iron offsets
mag_x_off = -29.55
mag_y_off = -62.65
mag_z_off = -66.55
#Initialize state vector. transpose([qs,qx,qy,qz,bx,by,bz]) 
X_k = numpy.transpose(numpy.array([1.0,0.0,0.0,0.0,-0.758,1.401,0.504]))
#Initialize Q_k, the error covariance matrix
Q_k_prev = numpy.array([[0.0001,0,0,0,0,0,0], #
                  [0,0.0001,0,0,0,0,0],
                  [0,0,0.0001,0,0,0,0],
                  [0,0,0,0.0001,0,0,0],
                  [0,0,0,0,0.038,0,0],
                  [0,0,0,0,0,0.024,0],
                  [0,0,0,0,0,0,0.015]])
#Initialize R_k, the noise covariance matrix
R_k = numpy.array([[0.367,0,0],
                  [0,0.719,0],
                  [0,0,0.269]])
#Initialize N_k, the noise model covariance matrix
P_k = numpy.array([[10**(-8),0,0,0,0,0,0],
                  [0,10**(-8),0,0,0,0,0],
                  [0,0,10**(-8),0,0,0,0],
                  [0,0,0,10**(-8),0,0,0],
                  [0,0,0,0,10**(-8),0,0],
                  [0,0,0,0,0,10**(-8),0],
                  [0,0,0,0,0,0,10**(-8)]])
#Initialize rotation variables
p = 0 #gyro x output 
q = 0 #gyro y output
r = 0 #gyro z output
omega_x = p - X_k[4] #correct for bias
omega_y = q - X_k[5] #correct for bias
omega_z = r - X_k[6] #correct for bias
#Initialize quaternion variables
#qs = X_k[0]
#qx = X_k[1]
#qy = X_k[2]
#qz = X_k[3]

#Initialize time step. Change this with the frequency that the algorithm runs at
delta_t = 0.1
while True:
    # Read acceleration & magnetometer.
    accel_x, accel_y, accel_z = sensor.accelerometer
    mag_x, mag_y, mag_z = sensor.magnetometer
    p, q, r = sensor2.gyroscope
    #Subtract magnetometer hard offsets
    mag_x = mag_x - mag_x_off
    mag_y = mag_y - mag_y_off
    mag_z = mag_z - mag_z_off
    #Get angular rates
    omega_x = p - X_k[4] #correct for bias
    omega_y = q - X_k[5] #correct for bias
    omega_z = r - X_k[6] #correct for bias
    
    #Get quat values
    qs = X_k[0]
    qx = X_k[1]
    qy = X_k[2]
    qz = X_k[3]
    
    #Create array for multiplication to determine next quaternion
    #omega_mat = numpy.array([[0.0, -p, -q, -r],[p, 0.0, r, -q]
    #                         [q, -r, 0.0, p], [r, q, -p, 0.0]])
    #Determine jacobian matrix
    delta_f_delta_X = numpy.array([[0,-omega_x,-omega_y,-omega_z,qx,qy,qz],
                              [omega_x,0,omega_z,-omega_y,-qs,qz,-qy],
                              [omega_y,-omega_z,0,omega_x,-qz,-qs,qx],
                              [omega_z,omega_y,-omega_x,0,qy,-qx,-qs],
                              [0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0]])
    
    #Initialize quaternion to Euler angle Taylor series expansion equations
    delta_phi_delta_qs = 2*qx*(-2*(qy*qy+qx*qx)+1)/(4*(qy*qz+qs*qx)*(qy*qz+qs*qx)+(-2*(qy*qy+qx*qx)+1)*(-2*(qy*qy+qx*qx)+1))
    delta_phi_delta_qx = 2*(2*qs*qx*qx+4*qy*qz*qx+qs-2*qy*qy*qs)/(4*(qy*qz+qs*qx)*(qy*qz+qs*qx)+(1-2*(qx*qx+qy*qy))*(1-2*(qx*qx+qy*qy)))
    delta_phi_delta_qy = 2*(qz-2*q*qx*qx+2*qy*qy*qz+4*qy*qs*qx)/(4*(qy*qz+qs*qx)*(qy*qz+qs*qx)+(1-2*(qx*qx+qy*qy))*(1-2*(qx*qx+qy*qy)))
    delta_phi_delta_qz = 2*qy*(-2*(qy*qy+qx*qx)+1)/(4*(qy*qz+qs*qx)*(qy*qz+qs*qx)+(-2*(qy*qy+qx*qx)+1)*(-2*(qy*qy+qx*qx)+1))
    delta_theta_delta_qs = 2*qy/math.sqrt(1-4*(qx*qz-qs*qy)*(qx*qz-qs*qy))
    delta_theta_delta_qx = -2*qz/math.sqrt(1-4*(qx*qz-qs*qy)*(qx*qz-qs*qy))
    delta_theta_delta_qy = 2*qs/math.sqrt(1-4*(qx*qz-qs*qy)*(qx*qz-qs*qy))
    delta_theta_delta_qz = -2*qx/math.sqrt(1-4*(qx*qz-qs*qy)*(qx*qz-qs*qy))
    delta_psi_delta_qs = 2*qz*(-2*(qy*qy+qz*qz)+1)/(4*(qx*qy+qs*qz)*(qx*qy+qs*qz)+(-2*(qy*qy+qz*qz)+1)*(-2*(qy*qy+qz*qz)+1))
    delta_psi_delta_qx = 2*qy*(-2*(qy*qy+qz*qz)+1)/(4*(qx*qz+qs*qz)*(qx*qz+qs*qz)+(-2*(qy*qy+qz*qz)+1)*(-2*(qy*qy+qz*qz)+1))
    delta_psi_delta_qy = 2*(2*qx*qy*qy+4*qy*qs*qz+qx-2*qx*qz*qz)/(4*(qx*qy+qs*qz)*(qx*qy+qs*qz)+(1-2*(qy*qy+qz*qz))*(1-2*(qy*qy+qz*qz)))
    delta_psi_delta_qz = 2*(2*qs*qz*qz+4*qx*qy*qz+qs-2*qy*qy*qs)/(4*(qx*qy+qs*qz)*(qx*qy+qs*qz)+(1-2*(qy*qy+qz*qz))*(1-2*(qy*qy+qz*qz)))
    
    #Store these in a Jacobian matrix
    A_k = numpy.array([[delta_phi_delta_qs,delta_phi_delta_qx,delta_phi_delta_qy,delta_phi_delta_qz,0,0,0],
                      [delta_theta_delta_qs,delta_theta_delta_qx,delta_theta_delta_qy,delta_theta_delta_qz,0,0,0],
                      [delta_psi_delta_qs,delta_psi_delta_qx,delta_psi_delta_qy,delta_psi_delta_qz,0,0,0]])
    #Create transition matrix from Jacobian
    T_k = numpy.add(numpy.identity(7),numpy.matmul(numpy.multiply(0.5*delta_t,delta_f_delta_x),A_k))
    #Predicted X
    X_predict = numpy.matmul(T_k,X_k)
    #Predicted Q
    Q_predict = numpy.add(numpy.matmul(numpy.matmul(T_k,Q_k_prev),numpy.transpose(T_k)),P_k)
    #Calculate Kalman gain
    K_k = numpy.matmul(numpy.matmul(Q_predict,numpy.transpose(A_k)),
                         numpy.linalg.inv(numpy.add(numpy.matmul(numpy.matmul(A_k,Q_predict),numpy.transpose(A_k),R_k))))
    #Calculate Euler angles based on observed acceleration/mag values
    phi_acc = math.atan2(-accel_y/accel_z)
    theta_acc = math.asin(-accel_x/math.sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z))
    psi_acc = math.atan2(mag_y/mag_x)
    #Store these in z_k, the actual measurement array
    z_k = numpy.transpose(numpy.array([phi_acc,theta_acc,psi_acc]))
    #Calculate nonlinear gyro Euler angles
    phi_nl = math.atan2(2*(qy*qz+qs*qx)/(1-2*(qx*qx+qy*qy)))
    theta_nl = -math.asin(2*(qx*qz-qs*qy))
    psi_nl = math.atan2(2*(qx*qy+qs*qz)/(1-2*(qy*qy+qz*qz)))
    #Store these in h_k
    h_k = numpy.transpose(numpy.array([phi_nl,theta_nl,psi_nl]))
    #Generate innovation matrix
    y_k = numpy.subtract(z_k,h_k)
    #Update state vector
    X_k = numpy.add(X_predict,numpy.dot(K_k,y_k))
    #Normalize quaternion values
    norm = math.sqrt(X_k[0]*X_k[0]+X_k[1]*X_k[1]+X_k[2]*X_k[2]+X_k[3]*X_k[3])
    X_k[0] = X_k[0]/norm
    X_k[1] = X_k[1]/norm
    X_k[2] = X_k[2]/norm
    X_k[3] = X_k[3]/norm
    #Update error covariance matrix
    Q_k = numpy.subtract(Q_predict,numpy.matmul(numpy.matmul(K_k,A_k),Q_predict))
    #Print out currently read and predicted 
    print(["Acc: Phi: %s Theta: %s Psi: %s " % (phi_acc,theta_acc,psi_acc)])
    print(["Kalman: Phi: %s Theta: %s Psi: %s " % (phi_nl,theta_nl,psi_nl)])
    # Delay for delta_t.
    time.sleep(delta_t)

