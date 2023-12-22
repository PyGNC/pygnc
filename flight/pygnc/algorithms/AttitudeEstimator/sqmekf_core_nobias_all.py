#from autograd import jacobian, numpy as np
import autograd.numpy as np
#from scipy.linalg import sqrtm 
#from scipy.linalg import qr
from scipy.linalg import solve
from scipy.linalg import block_diag
import math
from scipy.linalg import sqrtm
from scipy.linalg import qr
#(TODO: implement the square root version of the MEKF to see if it improves the results?)

#import a set of utils for mekf
from .mekf_utils import *

# The state vector is defined as follows:
# x[0], x[1], x[2], x[3] -> quaternion
# x[4], x[5], x[6] -> x,y,z gyro bias
# x[7], x[8], x[9] -> x,y,z magnetometer bias

#values of C are really small. may be worth implementing the square root version of the MEKF

class SQMEKFCore_nb_all:
    # constructor
    #def __init__(self, P0, dynamics, gyro_m, measure, R, Q) -> None:
    def __init__(self, F0, dynamics, gyro_m, measure, Q) -> None:
        self.F = F0  # initial covariance
        self.f = dynamics  # discrete dynamics function used
        self.u = gyro_m #gyro measurements
        self.g = measure  # measurement function used
        #self.R = R  # measurement noise
        self.sqrt_Q = sqrtm(Q) #process noise

    def initialize_state(self, x0):
        self.x = x0

    #9x1 measurements vectors [6 lux measurements; 3 b vector measurents]
    #revised get R
    def get_R_notworking(self, all_raw_measurements): 
        
        #positive face lux measurements
        sp = all_raw_measurements[0:3][:,np.newaxis]

        #negative face lux measurements
        sn = all_raw_measurements[3:6][:,np.newaxis]

        #magnetometer measurements
        mag = all_raw_measurements[6:][:,np.newaxis]

        #0.01 is the standard deviation from the lux measurements. Standard deviation from models.jl definition
        R_lux = np.eye(6)*(0.01)**2

        #transform 6x6 lux uncertainty to 3 dimensional vector noise

        #jacobian for the linear covariance transformation
        #each a 3x3 matrix of the partials. df/dsp and df/dsn
        A_lux_1 = (np.linalg.norm(sp-sn)*np.eye(3) - (1/np.linalg.norm(sp-sn))*(sp-sn)@(sp-sn).T@(np.eye(3)))/(np.linalg.norm(sp-sn)**2)
        A_lux_2 =  (np.linalg.norm(sp-sn)*(-1*np.eye(3)) - (1/np.linalg.norm(sp-sn))*(sp-sn)@(sp-sn).T@(-1*np.eye(3)))/(np.linalg.norm(sp-sn)**2)

        A_lux = np.hstack((A_lux_1, A_lux_2))

        R_sun_vec = A_lux@R_lux@A_lux.T

        #do the same for the magnetometer measurement. Standard deviation from models.jl definition
        #trying out. kinda works
        #R_mag = np.eye(3)*(40.6)**2

        #standard deviation of magnetometer from models.jl
        #0.6 from magnetometer measurement
        #40 from the bias
        R_mag = np.eye(3)*(0.6)**2

        #standard deviation of magnetometer bias from models.jl. in uT
        #R_mb = np.eye(3)*(40)**2

        #R_total = block_diag(R_mag, R_mb)

        A_mag = (np.linalg.norm(mag)*np.eye(3) - (mag/np.linalg.norm(mag))@mag.T)/(np.linalg.norm(mag)**2)

        #A_total = np.hstack((A_mag, A_mag))
        R_mag_vec = A_mag@R_mag@A_mag.T
        #R_mag_vec = A_total@R_total@A_total.T

        #measurement noise on inertial vectors
        R_noise = block_diag(R_sun_vec, R_mag_vec)

        return R_noise


    #old get R function. (wrong implementation)
    def get_R(self, all_raw_measurements): 
        
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
        #A_lux_2 =  (np.linalg.norm(sp-sn)*-1*np.eye(3) - (1/np.linalg.norm(sp-sn))*(sn-sp)@(sn-sp).T)/(np.linalg.norm(sp-sn)**2)

        A_lux_2 =  (np.linalg.norm(sp-sn)*-1*np.eye(3) - (-1/np.linalg.norm(sp-sn))*(sp-sn)@(sp-sn).T)/(np.linalg.norm(sp-sn)**2)

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
        #bk = self.x[4:7]
        #mb = self.x[7:10]
        uk = self.u
        theta = np.linalg.norm(uk)*h
        r = (uk)/np.linalg.norm(uk)
        deltaq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))

        #jacobian for just the quaternion and gyro bias
        # Ak11 = H.T@L(qk1[0:4]).T@L(qk[0:4])@R(deltaq)@H
        # Ak12 = -0.5*h*np.identity(3)
        # Ak21 = np.zeros((3,3))
        # Ak22 = np.identity(3)

        # Ak = np.block([[Ak11, Ak12],[Ak21, Ak22]])

        #jacobian for the quaternion, gyro bias, and magnetometer bias
        Ak11 = H.T@L(qk1[0:4]).T@L(qk[0:4])@R(deltaq)@H
        #Ak12 = -0.5*h*np.identity(3)
        #Ak13 = np.zeros((3,3))
        #Ak21 = np.zeros((3,3))
        #Ak22 = np.identity(3)
        #Ak23 = np.zeros((3,3))
        #Ak31 = np.zeros((3,3))
        #Ak32 = np.zeros((3,3))
        #Ak33 = np.identity(3)

        #Ak = np.block([[Ak11, Ak12, Ak13],[Ak21, Ak22, Ak23],[Ak31, Ak32, Ak33]])

        #Ak = np.block([[Ak11, Ak12],[Ak21, Ak22]])

        Ak = np.real(Ak11)
        #print("this is A11", Ak11)
        #print("this is A: ", Ak)

        return Ak

    def predict(self, h):
        """
        MEKF prediction step
        h is the timestep
        """
        
        #print("this is gyro measurement: ", self.u)
        # discrete dynamics function of state
        x_predicted = self.f(self.x,self.u,h)

        # find the jacobian in closed form
        #A = self.get_jacobian(self, h)

        A = self.get_jacobian(h)

        F = self.F

        n = np.vstack((F @ A.T, self.sqrt_Q))

        _, F_predicted = qr(np.real(n), mode="economic")

        #P_predicted = A @ self.P @ A.T + self.Q

        return x_predicted, F_predicted
    
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
        Z = body_measurement[:,np.newaxis] - predicted_body_measurement

        print("this is body measurement: ", body_measurement)
        print("this is predicted body: ", predicted_body_measurement)
        
        #print("this is Z: ", Z)

        #print("this is C: ", C)

        
        return Z, C
    
    def kalman_gain(self, F_predicted, C, all_raw_measurements):
        """
        MEKF Kalman Gain
        """

        #R = self.get_R(all_raw_measurements)
        #R = self.get_R_notworking(all_raw_measurements)

        R = block_diag(np.identity(3)*(5*math.pi/180)**2, np.identity(3)*(3*math.pi/180)**2)

        sqrt_R = sqrtm(R)

        m = np.vstack((F_predicted @ C.T, sqrt_R))

        _, G = qr(m, mode="economic")

        M = solve(G.T, C) @ F_predicted.T @ F_predicted

        L_inside = solve(G, M)

        L_ = L_inside.T

        # innovation covariance
        #S = C@P_predicted@C.T + R

        #Kalman gain 
        #L_ = np.linalg.solve(S.T, C @ P_predicted.T).T

        return L_

    #measurement is lux and gyro
    #measurment are the body vectors of the sun and magnetometer
    #inertial measurement are the unit vectors from sun and magnetometer
    def update(self, all_raw_measurements, body_measurement, inertial_measurement, dt):
        """
        MEKF update step
        """
        # Predict the next state and covariance
        x_predicted, F_predicted = self.predict(dt)

        #this y should be the inertial measurement
        Z, C = self.innovation(all_raw_measurements, body_measurement, inertial_measurement, x_predicted)

        # calculate kalman gain
        L_ = self.kalman_gain(F_predicted, C, all_raw_measurements)

        delta = L_ @ Z

        #go to vector 
        delta = np.real(delta[:,0])

        print("this is delta: ", delta)

        #get the delta quaternion from the rodrigues parameter (delta rotation)
        dq = RP_to_quaternion(delta[0:3])

        #print("this is dq: ", dq)
        #print("dq size: ", dq.shape)
        #print("size of Lq: ", L(x_predicted[0:4]).shape)
        #normalize
        dq = dq/np.linalg.norm(dq)

        #print("this is Lq: ", L(x_predicted[0:4]))
        #print("this is dq normalized...", dq)

        #print("multplication: ", L(x_predicted[0:4])@dq)

        new_x = L(x_predicted[0:4])@dq

        #print(
        #    "this is the new x: ", new_x
        #)
        #update the quaternion 
        #self.x[0:4] = L(x_predicted[0:4])@dq

        self.x = new_x
        
        #print("this is self.x: ", self.x[0:4])
        #print("this is self.x", self.x)
        #update the gyro bias
        #self.x[4:7] = x_predicted[4:7] + delta[3:6]

        #update the magnetometer bias
        #self.x[7:] = x_predicted[7:] + delta[6:]

        #get R
        #R = self.get_R(all_raw_measurements)
        #R = self.get_R_notworking(all_raw_measurements)

        #biases kinda converging
        R = block_diag(np.identity(3)*(5*math.pi/180)**2, np.identity(3)*(3*math.pi/180)**2)

        sqrt_R = sqrtm(R)
        #update the covariance
        #self.F = (np.identity(9) - L_@C)@F_predicted@(np.identity(9) - L_@C).T + L_@R@L_.T

        e = np.real(np.vstack((F_predicted @ (np.identity(3) - L_ @ C).T, sqrt_R @ L_.T)))

        # update the square root covariance
        _, new_F = qr(e, mode="economic")

        self.F = new_F

        #print("result: ", qr(e, mode="economic"))
        #print("this is self.F: ", self.F)