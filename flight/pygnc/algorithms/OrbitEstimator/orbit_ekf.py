import autograd.numpy as np
import brahe
from scipy.linalg import sqrtm
from scipy.linalg import qr
from .ekf_core import EKFCore


def atm_density(alt):
    """
    Expoential fit model for atmospheric density from from SMAD data.
    For orbits with an altitude of 350-650 km
    Input: altitude in km
    Output: density in kg/m^3
    """

    atm_d = np.exp(-1.63481928e-2 * alt) * np.exp(-2.00838711e1)

    return atm_d


def process_dynamics(x):
    """ "
    Process model for the EKF
    Input: state vector (x)
    Output: Time derivative of the state vector (x_dot)
    """

    R_EARTH = brahe.constants.R_EARTH  # m, radius of the Earth
    OMEGA_EARTH = brahe.constants.OMEGA_EARTH  # rad/s, angular velocity of the Earth
    μ = brahe.constants.GM_EARTH  # m3/s2, gravitational parameter of the Earth
    J2 = brahe.constants.J2_EARTH  # J2 perturbation constant

    # position
    q = np.array(x[0:3])

    # velocity
    v = np.array(x[3:6])

    # unmodeled accelerations
    a_d = np.array(x[6:9])

    # time correlation coefficients
    beta = np.array(x[9:12])

    # drag coefficient
    cd = 2.0

    # cross sectional area (m^2)
    A = 0.1

    # angular velocity of the Earth
    omega_earth = np.array([0, 0, OMEGA_EARTH])

    # relative velocity
    v_rel = v - np.cross(omega_earth, q)

    # get the altititude in km
    alt = (np.linalg.norm(q) - R_EARTH) * 1e-3

    # estimated rho from model
    rho_est = atm_density(alt)

    # drag force
    f_drag = -0.5 * cd * (A) * rho_est * np.linalg.norm(v_rel) * v_rel

    # Two body acceleration
    a_2bp = (-μ * q) / (np.linalg.norm(q)) ** 3

    # z unit vector
    Iz = np.array([0, 0, 1])

    # accleration due to J2
    a_J2 = ((3 * μ * J2 * R_EARTH**2) / (2 * np.linalg.norm(q) ** 5)) * (
        (((5 * np.dot(q, Iz) ** 2) / np.linalg.norm(q) ** 2) - 1) * q
        - 2 * np.dot(q, Iz) * Iz
    )

    # total acceleration (two body + J2 + drag + unmodeled accelerations)
    a = a_2bp + a_J2 + f_drag + a_d

    # unmodeled accelerations modeled as a first order gaussian process
    # time corellation coefficients modeled as a random walk (time derivative = 0)
    a_d_dot = -np.diag(beta) @ a_d

    # state derivative
    x_dot = np.concatenate([v, a, a_d_dot, np.zeros(3)])

    return x_dot


def rk4(x, h, f):
    """
    Runge-Kutta 4th order integrator
    """

    k1 = f(x)
    k2 = f(x + h / 2 * k1)
    k3 = f(x + h / 2 * k2)
    k4 = f(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


class OrbitEKF(EKFCore):
    """
    Defines the EKF for the orbit determination problem
    """

    def __init__(self):
        def time_dynamics(x, dt):
            """
            Discrete-time dynamics function
            """
            return rk4(x, dt, process_dynamics)

        def measurement_function(x):
            """
            Measurement function for GPS measurements providing position and velocity
            """
            C = np.hstack((np.eye(6), np.zeros((6, 6))))
            measurement = C @ x
            return measurement, C

        # standard deviation of the GPS position measurement in meters
        std_gps_measurement = 10

        # standard devation of the GPS velocity measurement in m/s
        std_velocity = 0.001
        # switch depending if we are using an accuracy of mm/s or cm/s on the velocity
        # std_velocity = 0.01

        # tuning parameters for the first order gauss markov model
        q_betas = 2e-9 * np.ones(3)
        q_eps = 5.5e-11 * np.ones(3)

        # measurement noise matrix
        R_measure = np.identity(6) * np.hstack(
            [
                (((std_gps_measurement) ** 2) / 3) * np.ones(3),
                ((std_velocity) ** 2) / 3 * np.ones(3),
            ]
        )

        # initial covariance matrix
        P0 = np.identity(12) * np.hstack(
            (
                np.ones(3) * ((std_gps_measurement) ** 2) / 3,
                np.ones(3) * ((std_velocity) ** 2) / 3,
                np.ones(3) * 5e-4**2,
                np.ones(3) * 4e-4**2,
            )
        )
        # initial square root covariance matrix
        F0 = sqrtm(P0)

        super().__init__(
            F0, time_dynamics, measurement_function, q_betas, q_eps, R_measure
        )

    def initialize_state(self, state_eci):
        # initial state
        x0 = np.concatenate([state_eci, np.zeros(3), 1e-3 * np.ones(3)])
        super().initialize_state(x0)

    def predict(self, dt):
        self.x, self.F = super().predict(dt)

    def update(self, y):
        """
        EKF update step with no prediction
        """
        # innovation step
        Z, C = self.innovation(y, self.x, self.F)

        # calculate kalman gain
        L = self.kalman_gain(self.F, C)

        # update the state
        self.x = self.x + L @ Z

        e = np.vstack((self.F @ (np.identity(12) - L @ C).T, self.sqrt_R @ L.T))

        # update the square root covariance
        _, self.F = qr(e, mode="economic")