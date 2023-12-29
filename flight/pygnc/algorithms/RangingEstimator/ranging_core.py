from autograd import jacobian
from autograd import numpy as np
from scipy.linalg import sqrtm
from scipy.linalg import qr
from scipy.linalg import solve
from scipy.optimize import least_squares
import cvxpy as cp

class BatchLSQCore:
    def __init__(self, xc,x0d, y, dynamics, measure, residuals, Q, R,dt, meas_gap, dt_goal) -> None:
        self.xc = xc # chief state
        self.x = x0d # deputy state
        self.y = y # measurements
        self.f = dynamics # discrete dynamics function used
        self.g = measure # measurement function used
        self.r = residuals # residuals function
        self.R = R # measurement noise
        self.Q = Q # process noise
        self.dt = dt
        self.meas_gap = meas_gap
        self.dt_goal = dt_goal

    def scipy_solver(self):
        res = least_squares(lambda x: self.r(self.xc,x, self.y, self.Q, self.R, self.dt, self.meas_gap, self.dt_goal), self.x.flatten(), method='lm')
        return res.x.reshape((6,-1))