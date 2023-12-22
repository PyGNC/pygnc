import autograd.numpy as np
from scipy.linalg import block_diag
import math
from .sqmekf_core import SQMEKFCore
from .mekf_utils import *

from scipy.linalg import sqrtm

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

    #biased gyro measurement in radians
    omega = u

    theta = np.linalg.norm(omega - beta)*h

    r = (omega - beta)/np.linalg.norm(omega-beta)

    dq = np.hstack(([np.cos(theta/2)], r*np.sin(theta/2)))

    qnext = L(q)@ dq

    #normalize the dq and try

    qnext = qnext/np.linalg.norm(qnext)

    print("norm of qnext: ", np.linalg.norm(qnext))

    x_next = np.hstack((qnext, beta, mag_bias))

    return x_next

class OrbitSQMEKF(SQMEKFCore):
    """
    Defines the EKF for the orbit determination problem
    """

    def __init__(self):

        def time_dynamics(x, u, dt):
            """
            Discrete-time dynamics function
            """
            return process_dynamics(x, u, dt)
        
        #take in the state and an inertial measurement at that timestep
        def measurement_function(x, inertial_measurement):
            """
            Measurement function for magnetic field and 
            sun vector measurements in the ECI frame. The predicted
            measurement is this true measurment rotated by the 
            predicted attitude
            """
            #sun is 0:3
            #magnetometer is 3:6

            q = x[0:4]
            
            #magnetomter bias in the body frame
            magnetometer_bias = x[7:10][:,np.newaxis]

            #takes us from body to imu frame
            #R_IMU_body = np.array([[0.0, 1.0, 0.0],
            #          [-1.0, 0.0, 0.0],
            #          [0.0, 0.0, -1.0]]).T

            #inertial measurements (not normalized)
            #b_measurement = inertial_measurement[0:3][:,np.newaxis]
            #sun_measurement = inertial_measurement[3:][:,np.newaxis]

            sun_measurement = inertial_measurement[0:3][:,np.newaxis]

            b_measurement = inertial_measurement[3:][:,np.newaxis]

            #b_measurement = inertial_measurement[3:][:,np.newaxis]

            #this is rotation from inertial to body frame
            Q_N_B = quaternion_to_rotmatrix(q).T 

            #need the unnormalized predictions for the jacobians
            sun_predicted = Q_N_B @ sun_measurement

            mag_predicted = Q_N_B @ b_measurement

            #normalized predictions. This is what we use in the measurement model. 
            sun_predicted_n = Q_N_B @ (sun_measurement/np.linalg.norm(sun_measurement))

            mag_predicted_n = (Q_N_B@b_measurement+magnetometer_bias)/np.linalg.norm((Q_N_B@b_measurement+magnetometer_bias))

            predicted_measurement = np.vstack((sun_predicted_n, mag_predicted_n))
            
            #Find C -> dh/dx
            #3x3 matrices 
            #the backslash is just to start a new line in the equation

            #NEW REVISED JACOBIANS (working)
            dy_sun_dphi = (2*np.linalg.norm(Q_N_B@sun_measurement)*hat(np.squeeze(sun_predicted)) -\
                        ((Q_N_B@sun_measurement)/np.linalg.norm(Q_N_B@sun_measurement))@sun_measurement.T\
                        @Q_N_B.T@(2*hat(np.squeeze(sun_predicted))))/(np.linalg.norm(Q_N_B@sun_measurement)**2)

            #3x3 matrices
            dy_mag_dphi = (2*np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)*hat(np.squeeze(mag_predicted)) -\
                        ((Q_N_B@b_measurement + magnetometer_bias)/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)))\
                        @(Q_N_B@b_measurement + magnetometer_bias).T @ (2*hat(np.squeeze(mag_predicted))))/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)**2)

            dy_mag_dmb = (np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)*np.eye(3) -\
                        ((Q_N_B@b_measurement + magnetometer_bias)/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)))\
                        @ (Q_N_B@b_measurement + magnetometer_bias).T @ np.eye(3))/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)**2)

            #OLD (has bugs)
            # dy_sun_dphi_old = (2*np.linalg.norm(Q_N_B@sun_measurement)*hat(np.squeeze(sun_predicted)) -\
            #             ((Q_N_B@sun_measurement)/np.linalg.norm(Q_N_B@sun_measurement))@sun_measurement.T\
            #             @Q_N_B.T@hat(np.squeeze(sun_predicted)))/(np.linalg.norm(Q_N_B@sun_measurement)**2)

            #3x3 matrices
            # dy_mag_dphi_old = (2*np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)*hat(np.squeeze(mag_predicted)) -\
            #             ((Q_N_B@b_measurement + magnetometer_bias)/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)))\
            #             @(Q_N_B@b_measurement + magnetometer_bias).T + hat(np.squeeze(mag_predicted)))/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)**2)

            # dy_mag_dmb_old = (np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)*np.eye(3) -\
            #             ((Q_N_B@b_measurement + magnetometer_bias)/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)))\
            #             @ (Q_N_B@b_measurement + magnetometer_bias).T @ np.eye(3))/(np.linalg.norm(Q_N_B@b_measurement + magnetometer_bias)**2)

            # C_old = np.block([[dy_sun_dphi_old, np.zeros((3,6))], [dy_mag_dphi_old, np.zeros((3,3)), dy_mag_dmb_old]])

            C = np.block([[dy_sun_dphi, np.zeros((3,6))], [dy_mag_dphi, np.zeros((3,3)), dy_mag_dmb]])

            print("RANK OF C: ", np.linalg.matrix_rank(C))
            #print("this is C: ", C)
            #print("this is C_old: ", C_old)
            #b_measurement_biased = b_measurement + magnetometer_bias
            #inertial_measurement = np.vstack((sun_measurement, b_measurement_biased))
            #inertial_measurement  = [sun; b measurement biased]
            #Q_m = block_diag(quaternion_to_rotmatrix(q).T,quaternion_to_rotmatrix(q).T)
            #predicted_measurement = Q_m @ inertial_measurement #+ np.vstack((np.zeros((3,3)), np.eye(3)))
            #updated C matrix to incorporate magnetometer bias. Ensure magnetomoeter bias is last in the measurement vector
            #had before
            #C = np.block([[2*hat(predicted_measurement[0:3]), np.zeros((3,6))], [2*hat(predicted_measurement[3:6]), np.zeros((3,3)), np.identity(3)]])
            #drm_dphi = 2*np.linalg.norm(quaternion_to_rotmatrix(q).T@b_measurement + magnetometer_bias)@hat(predicted_measurement[3:6])
            #update (need to check if it works)
            #C = np.block([[2*hat(predicted_measurement[0:3]), np.zeros((3,6))], [2*hat(predicted_measurement[3:6]), np.zeros((3,3)), quaternion_to_rotmatrix(q).T]])
            
            return predicted_measurement, C
        
        #measurement noise matrix

        #NEED TO TUNE THIS. How do the standard deviation of the measurements affect the vector measurements?
        #std for the sun sensor lux values
        #From models.jl
        # std of sun sensor: 0.1 lux
        # std of magnetometer: 0.6 uT 
        #R_noise = block_diag(np.identity(3)*(5*math.pi/180)**2, np.identity(3)*(8*math.pi/180)**2)

        #the measurements of lux are 6 and magnetometer are 3. The uncertainty of these variables is mapped
        # to the unit vector uncertainty through a jacobian. A_m*P*A_m.T

        #process noise matrix
        #Q_noise = block_diag(np.identity(3)*(5e-4)**2, np.identity(3)*(0.00001)**2)
        #can be tuned
        #Q_noise = block_diag(np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(0.0001*math.pi/180)**2)
        #first noise is gyro noise, second is gyro bias random walk noise, and third is magnetometer bias random walk noise
        #Kind of working this bottom Q
        #Q_noise = block_diag(np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(0.001*math.pi/180)**2, np.identity(3)*(0.001*math.pi/180)**2)

        #all from models.jl
        #std of gyro: 0.025 deg/s
        #std of gyro bias: 3 deg/s
        #std of magnetometer bias: 40 uT
        #from the noise
        #Q_noise = block_diag(np.identity(3)*(0.025*math.pi/180)**2, np.identity(3)*(3*math.pi/180)**2, np.identity(3)*(40**2))

        #Q_noise = block_diag(np.identity(3)*(0.025*math.pi/180)**2, np.identity(3)*(3*math.pi/180)**2, np.identity(3)*(300**2))

        #testing from derivation. the 5 is the dt

        #from tutorial
        dt = 5
        # term1 = (0.025*math.pi/180)**2 * dt
        # term1_1 = (3*math.pi/180)**2 * (dt**3)/3
        # term1_2 = (3*math.pi/180)**2 * (dt**2)/2
        # term2_2 = (3*math.pi/180)**2 * dt
        # #should be 40
        # term3_3 = (40**2) * dt

        #tuning from tutorial 
        term1 = (0.05*math.pi/180)**2 * dt
        #having 1 and 40 here gets the symmetrical plot when the ground truth is plotted inversely
        term1_1 = (6*math.pi/180)**2 * (dt**3)/3
        term1_2 = (6*math.pi/180)**2 * (dt**2)/2
        term2_2 = (6*math.pi/180)**2 * dt
        #should be 40
        term3_3 = (80**2) * dt

        Q_noise = block_diag(np.identity(3)*(term1 + term1_1), np.identity(3)*(term2_2), np.identity(3)*(term3_3))
        Q_noise[0:3, 3:6] = np.identity(3)*(term1_2)
        Q_noise[3:6, 0:3] = np.identity(3)*(term1_2)

        Q_noise = Q_noise #increase process noise

        #Q_noise = np.hstack((np.identity(3)*((0.025**2)*5 + (3*math.pi/180)**2 * (5**3)/3, np.identity(3)*3**2 * (5**2)/2)))

        #Q_noise = block_diag(np.identity(3)*((0.025**2)*5 + (3*math.pi/180)**2 * (5**3)/3), np.identity(3)*(3*math.pi/180)**2*(5), np.identity(3)*(40**2)*(5))

        #testing
        #Q_noise = block_diag(np.identity(3)*(0.25*math.pi/180)**2, np.identity(3)*(15*math.pi/180)**2, np.identity(3)*(160**2))


        #trying out new Q
        #Q_noise = block_diag(np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(0.1*math.pi/180)**2, np.identity(3)*(5)**2)
        #initial covariance 

    
        #P_0 = np.identity(9)
        #Initialize with Q_noise
        P_0 = Q_noise

        F_0 = sqrtm(P_0)
        #initialize the MEKF
        #gyro initialized to zero and then update based off reading the
        #.bin files

        # super().__init__(
        #     P_0, time_dynamics, np.zeros(3), measurement_function, R_noise, Q_noise
        # )

        #removed R from the class 
        super().__init__(
            F_0, time_dynamics, np.zeros(3), measurement_function, Q_noise
        )

    def initialize_state(self):
        # initial state
        #arbitrary initial state. may need to change this
        #this is true value [1.000e+00 8.727e-04 7.775e-10 6.478e-08]
        #testing
        q0 = np.array([0.18257418583505536, 0.3651483716701107, 0.5477225575051661,0.7302967433402214])
        
        #initializing with vector part equal to zero doesn't work
        #q0 = np.array([-1, 0, 0,0])

        #b0 = np.array([0.001,0.001,0.001])
        #b0 = np.array([0.05,0.05,0.05]) #this is in rad/s
        b0 = np.array([1,1,1])*math.pi/180  #this is in rad/s
        #mb0 = np.array([0.001,0.001,0.001])
        mb0 = np.array([20,10,-25]) #this is in uT
        x0 = np.hstack((q0,b0, mb0))
        super().initialize_state(x0)