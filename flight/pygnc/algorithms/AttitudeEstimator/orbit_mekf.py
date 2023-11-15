import autograd.numpy as np
from scipy.linalg import block_diag
import math
from .mekf_core import MEKFCore
from .mekf_utils import *


def process_dynamics(self, h):
    """ "
    Process model for the MEKF
    Input: state vector (x)
    Output: Time derivative of the state vector (x_dot)
    Discrete Dynamics model 
    """

    #quaternion
    q = self.x[0:4]

    #gyro bias
    beta = self.x[4:7]

    #bias gyro measurement
    omega = self.u

    theta = np.linalg.norm(omega - beta)*h

    r = (omega - beta)/np.linalg.norm(omega-beta)

    dq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))
    
    qnext = L(q)@ dq

    x_next = np.hstack((qnext, beta))

    return x_next


class OrbitMEKF(MEKFCore):
    """
    Defines the EKF for the orbit determination problem
    """

    def __init__(self):

        def time_dynamics(x, dt):
            """
            Discrete-time dynamics function
            """
            return process_dynamics(self, dt)
        
        def measurement_function(x, inertial_measurement):
            """
            Measurement function for magnetic field and 
            sun vector measurements in the ECI frame. The predicted
            measurement is this true measurment rotated by the 
            predicted attitude
            """

            q = x[0:4]

            Q_m = block_diag(quaternion_to_rotmatrix(q).T,quaternion_to_rotmatrix(q).T)

            C = np.hstack((np.eye(6), np.zeros((6, 6))))

            measurement = C @ x

            return measurement, C

        #measurement noise matrix
        R_noise = block_diag(np.identity(3)*(8*math.pi/180)**2, np.identity(3)*(5*math.pi/180)**2)

        #process noise matrix
        Q_noise = block_diag(np.identity(3)*(5e-4)**2, np.identity(3)*(0.00001)**2)

        #initial covariance
        P_0 = np.identity(6)*1e-6

        #initialize the MEKF
        #gyro initialized to zero and then update based off reading the
        #.bin files

        super().__init__(
            P_0, time_dynamics, np.zeros(3), measurement_function, R_noise, Q_noise
        )


    def initialize_state(self, state_eci):
        # initial state
        #arbitrary initial state
        q0 = np.array([0.18257418583505536, 0.3651483716701107, 0.5477225575051661,0.7302967433402214])
        b0 = np.array([0.001,0.001,0.001])
        x0 = np.hstack((q0,b0))
        super().initialize_state(x0)
