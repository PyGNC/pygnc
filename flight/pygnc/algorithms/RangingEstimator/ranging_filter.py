from ranging_filter import RangingFilter
import autograd.numpy as np
from scipy.linalg import sqrtm
import brahe

from ranging_core import BatchLSQCore

R_EARTH = brahe.constants.R_EARTH  # m, radius of the Earth
OMEGA_EARTH = brahe.constants.OMEGA_EARTH  # rad/s, angular velocity of the Earth
μ = brahe.constants.GM_EARTH  # m3/s2, gravitational parameter of the Earth
J2 = brahe.constants.J2_EARTH  # J2 perturbation constant

def atm_density(alt):
    """
    Expoential fit model for atmospheric density from from SMAD data. 
    For orbits with an altitude of 350-650 km 
    Input: altitude in km
    Output: density in kg/m^3
    """

    atm_d = np.exp(-1.63481928e-2*alt) * np.exp(-2.00838711e1)

    return atm_d

def process_dynamics(x):
    """"
    Process model for the EKF
    Input: state vector (x)  
    Output: Time derivative of the state vector (x_dot)
    """
    # position
    q = np.array(x[0:3])

    #velocity
    v = np.array(x[3:6])

    #unmodeled accelerations
    # a_d = np.array(x[6:9])

    #time correlation coefficients
    # beta = np.array(x[9:12])

    # drag coefficient
    cd = 2.0

    # cross sectional area (m^2)
    A = 0.1

    # angular velocity of the Earth
    omega_earth = np.array([0, 0, OMEGA_EARTH])
    # relative velocity
    v_rel = v - np.cross(omega_earth, q)

    # get the altititude in km
    alt = (np.linalg.norm(q) - R_EARTH)*1e-3

    # estimated rho from model
    rho_est = atm_density(alt)

    # drag force
    f_drag = -0.5*cd*(A)*rho_est*np.linalg.norm(v_rel)*v_rel

    # gravitational parameter of the Earth
    μ = 3.986004418e14  # m3/s2

    # J2 perturbation constant
    J2 = 1.08264e-3

    #Two body acceleration
    a_2bp = (-μ*q)/(np.linalg.norm(q))**3

    #z unit vector
    Iz = np.array([0, 0, 1])

    #accleration due to J2
    a_J2 = ((3*μ*J2*R_EARTH**2)/(2*np.linalg.norm(q)**5)) * \
        ((((5*np.dot(q, Iz)**2)/np.linalg.norm(q)**2)-1)*q - 2*np.dot(q, Iz)*Iz)

    #total acceleration (two body + J2 + drag + unmodeled accelerations)
    a = a_2bp + a_J2 + f_drag #+ a_d

    #unmodeled accelerations modeled as a first order gaussian process
    #time corellation coefficients modeled as a random walk (time derivative = 0)
    # a_d_dot = -np.diag(beta)@a_d

    #state derivative
    x_dot = np.concatenate([v, a])#, a_d_dot, np.zeros(3)])

    return x_dot

def rk4(x, h, f):
    """
    Runge-Kutta 4th order integrator
    """
    
    k1 = f(x)
    k2 = f(x + h/2 * k1)
    k3 = f(x + h/2 * k2)
    k4 = f(x + h * k3)
    return x + h/6 * (k1 + 2*k2 + 2*k3 + k4)

def rk4_multistep(x, h, f, N):
    """
    Runge-Kutta 4th order integrator for multiple timesteps between propagated points
    """
    x_new = x
    for i in range(N):
        x_new = rk4(x_new, h, f)
    return x_new

def calculate_dt(meas_gap,dt_goal):
    """
    Calculate the timestep for the batch least squares solver
    """
    N = np.ceil(meas_gap/dt_goal).astype(int)
    dt = meas_gap/N
    return dt, N

class BA(BatchLSQCore):
    """
    Defines the Batch Least-Squares solver for the orbit determination problem
    """

    def __init__(self,xc, x0d,y,dt,meas_gap,dt_goal):

        def time_dynamics(x, dt, N):
            """"
            Batch dynamics function
            """
            x_dyn = np.zeros_like(x)
            for i in range(x.shape[1]):
                x_new = x[:,i]
                for j in range(N):
                    x_new = rk4(x_new, dt, process_dynamics)
                x_dyn[:,i] = x_new
            return x_dyn
        
        def measurement_function_single(xc,xd):
            """
            Measurement function for GPS measurements providing position and velocity, provides ranges between chief and deputy
            """
            measurement_range = np.array([np.linalg.norm(xc[0:3] - xd[0:3])])#,np.linalg.norm(xc[0:3] - xd[6:9]),np.linalg.norm(xc[0:3] - xd[12:15])])
            return measurement_range
        
        def measurement_function(xc,xd):
            """
            batch measurement function
            """
            x_meas = np.zeros(xd.shape[1])
            for i in range(xd.shape[1]):
                x_meas[i] = measurement_function_single(xc[:,i], xd[:,i])
            return x_meas
        
        def residuals(xc, xd,y,Q,R,dt,meas_gap, dt_goal):
            ############################################################
            #generate residuals of dynamics and measurement            #
            #estimate of the orbit of multiple satellites              #
            ############################################################
            xd  = xd.reshape((6, -1))
            xc = xc.reshape((6, -1))
            Q = sqrtm(np.linalg.inv(Q))
            R = np.sqrt(R)
            dyn_res_d1 = np.array([])
            meas_res = np.array([])
            N = meas_gap
            for i in range(xd.shape[1]-1):

                dyn_res_d1i = Q@(xd[0:6,i+1] - rk4_multistep(xd[0:6,i],dt_goal,process_dynamics,N))
                dyn_res_d1 = np.hstack((dyn_res_d1, dyn_res_d1i))
                meas_resi = R*(measurement_function_single(xc[:,i],xd[:,i]) - y[:,i])
                meas_res = np.hstack((meas_res, meas_resi))
            dyn_res_d1 = dyn_res_d1.reshape((6, xd.shape[1]-1))
            meas_res = meas_res.reshape((1, xd.shape[1]-1))
            dyn_res = dyn_res_d1
            stacked_res = np.vstack((dyn_res, meas_res))
            return stacked_res.flatten()
    
        def residuals_sum(xc,xd,y,Q,R,dt):
            res = residuals(xc,xd,y,Q,R,dt)
            return np.sum(res)

        #standard deviation of the GPS position measurement in meters
        std_gps_measurement = 10

        #standard devation of the GPS velocity measurement in m/s
        std_velocity = 0.001
        #switch depending if we are using an accuracy of mm/s or cm/s on the velocity
        #std_velocity = 0.01

        # standard deviation of the range measurement in meters
        std_range = 10

        #tuning parameters for the first order gauss markov model
        # q_betas = 2e-9 * np.ones(3)
        # q_eps = 5.5e-11 * np.ones(3)

        # Process noise covariances
        pose_std_dynamics = 5#4e-6#*1e-3 #get to km
        velocity_std_dynamics = 0.04#8e-6 #*1e-3 #get to km/s

        #measurement noise matrix
        # R_measure = np.identity(3) * np.hstack([((std_range)**2)/3*np.ones(3)])
        R_measure = std_range**2

        #Process ovariance matrix for one satellite
        Q_proc = np.hstack((np.ones(3) * ((pose_std_dynamics)**2), np.ones(3) * ((velocity_std_dynamics)**2)))
        #Repeat Q_ind for all satellites
        Q_proc = np.diag(Q_proc)
        

        #initial state
        x0d = x0d
        xc=xc
        dt_goal = dt_goal
        meas_gap = meas_gap

        #initial measurement
        # y = measurement_function(x0)

        super().__init__(xc, x0d, y, time_dynamics,
                         measurement_function, residuals, Q_proc, R_measure,dt,meas_gap,dt_goal)