#from autograd import jacobian, numpy as np
import autograd.numpy as np
#from scipy.linalg import sqrtm 
#from scipy.linalg import qr
from scipy.linalg import solve
#(TODO: implement the square root version of the MEKF to see if it improves the results?)

#import a set of utils for mekf
from .mekf_utils import *

# The state vector is defined as follows:
# x[0], x[1], x[2], x[3] -> quaternion
# x[4], x[5], x[6] -> x,y,z gyro bias
# x[7], x[8], x[9] -> x,y,z magnetometer bias


class MEKFCore:
    # constructor
    #def __init__(self, P0, dynamics, gyro_m, measure, R, Q) -> None:
    def __init__(self, P0, dynamics, gyro_m, measure, R, Q) -> None:
        self.P = P0  # initial covariance
        self.f = dynamics  # discrete dynamics function used
        self.u = gyro_m #gyro measurements
        self.g = measure  # measurement function used
        #self.R = R  # measurement noise
        self.Q = Q #process noise

    def initialize_state(self, x0):
        self.x = x0

    #9x1 measurements vectors [6 lux measurements; 3 b vector measurents]
    def get_R(all_raw_measurements): 
        
        #positive face lux measurements
        sp = all_raw_measurements[0:3]

        #negative face lux measurements
        sn = all_raw_measurements[3:6]

        #magnetometer measurements
        mag = all_raw_measurements[6:]

        #0.01 is the standard deviation from the lux measurements. Standard deviation from models.jl definition
        R_lux = np.eye(6)*(0.01)**2

        #transform 6x6 lux uncertainty to 3 dimensional vector noise

        #jacobian for the linear covariance transformation
        #each a 3x3 matrix of the partials. df/dsp and df/dsn
        A_lux_1 = (np.linalg.norm(sp-sn)*np.eye(3) - (1/np.linalg.norm(sp-sn))*(sp-sn)@(sp-sn).T)/(np.linalg.norm(sp-sn)**2)
        A_lux_2 =  (np.linalg.norm(sp-sn)*-1*np.eye(3) - (1/np.linalg.norm(sp-sn))*(sn-sp)@(sn-sp).T)/(np.linalg.norm(sp-sn)**2)

        A_lux = np.hstack((A_lux_1, A_lux_2))

        R_sun_vec = A_lux@R_lux@A_lux.T

        #do the same for the magnetometer measurement. Standard deviation from models.jl definition
        R_mag = np.eye(3)*(0.6)**2

        A_mag = np.linalg.norm(mag)*np.eye(3) - (mag/np.linalg.norm(mag))@mag.T

        R_mag_vec = A_mag@R_mag@A_mag.T

        #measurement noise on inertial vectors
        R_noise = block_diag(R_sun_vec, R_mag_vec)

        return R_noise


    def get_jacobian(self, h):

        qk = self.x[0:4]
        qk1 = self.f(self.x, self.u, h)
        bk = self.x[4:7]
        mb = self.x[7:10]
        uk = self.u
        theta = np.linalg.norm(uk-bk)*h
        r = (uk-bk)/np.linalg.norm(uk-bk)
        deltaq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))

        #jacobian for just the quaternion and gyro bias
        # Ak11 = H.T@L(qk1[0:4]).T@L(qk[0:4])@R(deltaq)@H
        # Ak12 = -0.5*h*np.identity(3)
        # Ak21 = np.zeros((3,3))
        # Ak22 = np.identity(3)

        # Ak = np.block([[Ak11, Ak12],[Ak21, Ak22]])

        #jacobian for the quaternion, gyro bias, and magnetometer bias
        Ak11 = H.T@L(qk1[0:4]).T@L(qk[0:4])@R(deltaq)@H
        Ak12 = -0.5*h*np.identity(3)
        Ak13 = np.zeros((3,3))
        Ak21 = np.zeros((3,3))
        Ak22 = np.identity(3)
        Ak23 = np.zeros((3,3))
        Ak31 = np.zeros((3,3))
        Ak32 = np.zeros((3,3))
        Ak33 = np.identity(3)

        Ak = np.block([[Ak11, Ak12, Ak13],[Ak21, Ak22, Ak23],[Ak31, Ak32, Ak33]])

        return Ak

    def predict(self, h):
        """
        MEKF prediction step
        h is the timestep
        """
        # discrete dynamics function of state
        x_predicted = self.f(self.x,self.u,h)

        # find the jacobian in closed form
        #A = self.get_jacobian(self, h)

        A = self.get_jacobian(h)

        P_predicted = A @ self.P @ A.T + self.Q

        return x_predicted, P_predicted
    
    # g is the measurement function
    def innovation(self, all_raw_measurements, body_measurement, inertial_measurement, x_predicted):
        """
        MEKF Innovation Step.
        y is the true measurement
        x_predicted is the predicted state
        """
        # predicted measurement
        #pass in predicted and an inertial measurement 
        predicted_body_measurement, C = self.g(x_predicted, inertial_measurement)

        # innovation
        #true body measurement  - predicted body measurement
        Z = body_measurement - predicted_body_measurement
        
        return Z, C
    
    def kalman_gain(self, P_predicted, C, all_raw_measurements):
        """
        MEKF Kalman Gain
        """

        R = self.get_R(all_raw_measurements)

        # innovation covariance
        S = C@P_predicted@C.T + R

        #Kalman gain 
        L_ = np.linalg.solve(S.T, C @ P_predicted.T).T

        return L_

    #measurement is lux and gyro
    #measurment are the body vectors of the sun and magnetometer
    #inertial measurement are the unit vectors from sun and magnetometer
    def update(self, all_raw_measurements, body_measurement, inertial_measurement, dt):
        """
        MEKF update step
        """
        # Predict the next state and covariance
        x_predicted, P_predicted = self.predict(dt)

        #this y should be the inertial measurement
        Z, C = self.innovation(all_raw_measurements, body_measurement, inertial_measurement, x_predicted)

        # calculate kalman gain
        L_ = self.kalman_gain(P_predicted, C, all_raw_measurements)

        delta = L_ @ Z

        #get the delta quaternion from the rodrigues parameter (delta rotation)
        dq = RP_to_quaternion(delta[0:3])
        
        #normalize
        dq = dq/np.linalg.norm(dq)

        #update the quaternion 
        self.x[0:4] = L(x_predicted[0:4])@dq
        
        #update the gyro bias
        self.x[4:7] = x_predicted[4:7] + delta[3:6]

        #update the magnetometer bias
        self.x[7:] = x_predicted[7:] + delta[6:]

        #get R
        R = self.get_R(all_raw_measurement)

        #update the covariance
        self.P = (np.identity(9) - L_@C)@P_predicted@(np.identity(9) - L_@C).T + L_@R@L_.T