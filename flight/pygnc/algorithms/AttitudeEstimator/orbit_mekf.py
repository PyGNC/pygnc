import autograd.numpy as np
from scipy.linalg import block_diag
import math
from .mekf_core import MEKFCore
from .mekf_utils import *

def process_dynamics(x, u, h):
    """
    Process model for the MEKF
    Input: state vector (x)
    Output: Time derivative of the state vector (x_dot)
    Discrete Dynamics model 
    """

    #quaternion
    q = x[0:4]

    #gyro bias
    beta = x[4:7]

    mag_bias = x[7:10]

    #bias gyro measurement
    omega = u

    theta = np.linalg.norm(omega - beta)*h

    r = (omega - beta)/np.linalg.norm(omega-beta)

    dq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))
    
    qnext = L(q)@ dq

    x_next = np.hstack((qnext, beta, mag_bias))

    return x_next


class OrbitMEKF(MEKFCore):
    """
    Defines the EKF for the orbit determination problem
    """

    def __init__(self):

        def time_dynamics(x, u, dt):
            """
            Discrete-time dynamics function
            """
            return process_dynamics(x, u, dt)
        
        def measurement_function(x, inertial_measurement):
            """
            Measurement function for magnetic field and 
            sun vector measurements in the ECI frame. The predicted
            measurement is this true measurment rotated by the 
            predicted attitude
            """

            q = x[0:4]

            Q_m = block_diag(quaternion_to_rotmatrix(q).T,quaternion_to_rotmatrix(q).T)

            predicted_measurement = Q_m @ inertial_measurement

            #updated C matrix to incorporate magnetometer bias. Ensure magnetomoeter bias is last in the measurement vector
            C = np.block([[2*hat(predicted_measurement[0:3]), np.zeros((3,6))], [2*hat(predicted_measurement[3:6]), np.zeros((3,3)), np.identity(3)]])

            #this predicted measurement is already biased since the inertial measurement
            #is already biased
            
            return predicted_measurement, C
        
        #measurement noise matrix

        #first is magnetic field, second is sun vector measurement
        #std for the sun sensor lux values 
        R_noise = block_diag(np.identity(3)*(5*math.pi/180)**2, np.identity(3)*(8*math.pi/180)**2)

        #process noise matrix
        #Q_noise = block_diag(np.identity(3)*(5e-4)**2, np.identity(3)*(0.00001)**2)
        #can be tuned
        #Q_noise = block_diag(np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(0.0001*math.pi/180)**2)
        #first noise is gyro noise, second is gyro bias random walk noise, and third is magnetometer bias random walk noise
        Q_noise = block_diag(np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(0.001*math.pi/180)**2, np.identity(3)*(0.001*math.pi/180)**2)

        #initial covariance
        P_0 = np.identity(9)*1e-6

        #initialize the MEKF
        #gyro initialized to zero and then update based off reading the
        #.bin files

        super().__init__(
            P_0, time_dynamics, np.zeros(3), measurement_function, R_noise, Q_noise
        )

    def initialize_state(self):
        # initial state
        #arbitrary initial state. may need to change this
        q0 = np.array([0.18257418583505536, 0.3651483716701107, 0.5477225575051661,0.7302967433402214])
        b0 = np.array([0.001,0.001,0.001])
        mb0 = np.array([0.001,0.001,0.001])
        x0 = np.hstack((q0,b0, mb0))
        super().initialize_state(x0)