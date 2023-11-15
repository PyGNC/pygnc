#from autograd import jacobian, numpy as np

import autograd.numpy as np
#from scipy.linalg import sqrtm (TODO: implement the square root version of the MEKF to see if it improves the results)
#from scipy.linalg import qr
from scipy.linalg import solve

#import a set of utils for mekf
from .mekf_utils import *

# The state vector is defined as follows:
# x[0], x[1], x[2] -> delta attitude
# x[3], x[4], x[5] -> x,y,z gyro bias

class MEKFCore:
    # constructor
    def __init__(self, P0, dynamics, gyro_m, measure, R, Q) -> None:
        self.P = P0  # initial covariance
        self.f = dynamics  # discrete dynamics function used
        self.u = gyro_m #gyro measurements
        self.g = measure  # measurement function used
        self.R = R  # measurement noise
        self.Q = Q #process noise

    def initialize_state(self, x0):
        self.x = x0

    def get_jacobian(self, h):

        qk = self.x[0:4]
        qk1 = self.f(self.x, self.u, h)
        bk = self.x[4:]
        uk = self.u
        theta = np.linalg.norm(uk-bk)*h
        r = (uk-bk)/np.linalg.norm(uk-bk)
        deltaq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))

        Ak11 = H.T@L(qk1[0:4]).T@L(qk[0:4])@R(deltaq)@H
        Ak12 = -0.5*h*np.identity(3)
        Ak21 = np.zeros((3,3))
        Ak22 = np.identity(3)

        Ak = np.block([[Ak11, Ak12],[Ak21, Ak22]])

        return Ak

    def predict(self, h):
        """
        MEKF prediction step
        h is the timestep
        """
        # discrete dynamics function of state
        x_predicted = self.f(self.x, h)

        # find the jacobian in closed form
        A = self.get_jacobian(self, h)

        P_predicted = A @ self.P @ A.T + self.Q

        return x_predicted, P_predicted
    
    # g is the measurement function
    def innovation(self, y, x_predicted):
        """
        MEKF Innovation Step.
        y is the true measurement
        x_predicted is the predicted state
        """

        # predicted measurement
        y_predicted, C = self.g(x_predicted)

        # innovation
        Z = y - y_predicted

        return Z, C
    
    def kalman_gain(self, P_predicted, C):
        """
        MEKF Kalman Gain
        """
        # innovation covariance
        S = C@P_predicted@C.T + self.R

        #Kalman gain 
        L_ = np.linalg.solve(S.T, C @ P_predicted.T).T

        return L_


    def update(self, y, dt):
        """
        MEKF update step
        """
        # Predict the next state and covariance
        x_predicted, P_predicted = self.predict(dt)

        # innovation step
        Z, C = self.innovation(y, x_predicted, P_predicted)

        # calculate kalman gain
        L_ = self.kalman_gain(P_predicted, C)

        delta = L_ @ Z

        #get the delta quaternion from the rodrigues parameter (delta rotation)
        dq = RP_to_quaternion(delta[0:3])
        
        #normalize
        dq = dq/np.linalg.norm(dq)

        #update the quaternion and gyro bias
        self.x[0:4] = L(x_predicted[0:4])@dq

        self.x[4:] = x_predicted[4:] + delta[3:]

        #update the covariance
        self.P = (np.identity(6) - L_@C)@P_predicted@(np.identity(6) - L_@C).T + L_@self.R@L_.T

