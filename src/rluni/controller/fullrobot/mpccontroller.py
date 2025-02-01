
import numpy as np
import importlib.resources as pkg_resources
import sympy as sp
from scipy.integrate import solve_ivp
import scipy.linalg
import cvxpy as cp
import time as clock
from collections import namedtuple

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils import call_super_first
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

torques = namedtuple("torques", ["roll", "pitch", "yaw"])

class MPCController(Controller):
    def __init__(
        self, dt, params=None, Qf=None, N=20, tau_max=1.0, warm_start=True, solver_kwargs={}
    ):
        """
        Model Predictive Controller class.

        Args:
        A (ndarray): Discrete system dynamics matrix x_{k+1} = A x_k + B u_k
        B (ndarray): Discrete input matrix x_{k+1} = A x_k + B u_k
        Q (ndarray): State cost matrix
        R (ndarray): Input cost matrix
        Qf (ndarray): Terminal state cost matrix (default: Q)
        N (int): Prediction horizon (default: 20)
        tau_max (float): Maximum control input (default: 1.0)

        """

        super().__init__()
        # A: roll_angle, roll_rate, motor_speed_roll
        # [       ] 
        # [  3x3  ]  
        # [       ]
        robot_params = {
            "g0": 9.81,
            "mw": 0.351,
            "mp": 1.670,
            "lp": 0.122,
            "lw": 0.18,
            "Ip": 0.030239,  # [kg * m^2]
            "Iw": 0.000768,
            "tau_max": tau_max,
        }
        self.params = params if params else robot_params
        self.g0 = self.params["g0"]
        self.mw = self.params["mw"]
        self.mp = self.params["mp"]
        self.lp = self.params["lp"]
        self.lw = self.params["lw"]
        self.Ip = self.params["Ip"]
        self.Iw = self.params["Iw"]
        self.m0 = self.g0 * (self.mp * self.lp + self.mw * self.lw)
        self.Inet = self.Ip + self.mp * self.lp**2 + self.Iw + self.mw * self.lw**2
        self.dt = dt

        self.A = np.array(
            [[0, 1, 0], [self.m0 / self.Inet, 0, 0], [-self.m0 / self.Inet, 0, 0]]
        )
        self.B = np.array(
            [[0], [-1 / self.Inet], [1 / self.Iw]]
        )
        self.n = self.A.shape[0]
        self.m = self.B.shape[1]
        self.A_tilde = np.eye(3) + self.A * self.dt
        self.B_tilde = self.B * self.dt
        self.Q = np.diag([1e4, 0.1, 25e-3])
        self.R = np.diag([100])
        self.w_final = 3.0
        self.Qf = self.w_final * self.Q
        self.tau_max = tau_max
        self.N = N

        # Build CVXPY problem structure once
        self._build_problem()

        # Store last solution for warm start optimization
        self.last_x_sol = None
        self.last_u_sol = None

        # Store solver kwargs
        self.solver_kwargs = solver_kwargs

    def _build_problem(self):
        # Create variables for optimization states and inputs
        self.x_var = cp.Variable((self.n, self.N + 1))
        self.u_var = cp.Variable((self.m, self.N))

        # Objective function with terminal cost
        cost = 0
        for k in range(self.N):
            cost += cp.quad_form(self.x_var[:, k], self.Q) + cp.quad_form(
                self.u_var[:, k], self.R
            )
        cost += cp.quad_form(self.x_var[:, self.N], self.Qf)

        # Set x0 as a problem parameter
        self.x0_param = cp.Parameter(shape=self.n, name="x0", value=np.zeros(self.n))

        # Define constraints
        self.constraints = []
        # Initial condition
        self.constraints.append(self.x_var[:, 0] == self.x0_param)
        # Dynamics and input constraints
        for k in range(self.N):
            self.constraints += [
                self.x_var[:, k + 1]
                == self.A_tilde @ self.x_var[:, k] + self.B_tilde @ self.u_var[:, k],
                -self.tau_max <= self.u_var[:, k],
                self.u_var[:, k] <= self.tau_max,
            ]

        # Define/compile the optimization problem
        self.prob = cp.Problem(cp.Minimize(cost), self.constraints)

    def solve_mpc(self, x0, warm_start=True):
        # Set the initial state parameter
        self.x0_param.value = x0

        # Start from last known solution if warm_start is enabled
        if warm_start and self.last_x_sol is not None and self.last_u_sol is not None:
            self.x_var.value = self.last_x_sol
            self.u_var.value = self.last_u_sol

        # Solve the problem
        self.prob.solve(solver=cp.CLARABEL, warm_start=warm_start, **self.solver_kwargs)

        # Store solution
        self.last_x_sol = self.x_var.value
        self.last_u_sol = self.u_var.value

        if self.u_var.value is None:
            print("No solution found!")
            return np.zeros(self.m)

        # Return the first control input
        return self.u_var[:, 0].value

    def get_control(self, t, x):
        # Solve the MPC problem
        # x = x[[0, 2, 3]]  # pare down to 3 states of interest
        return self.solve_mpc(x, warm_start=True)
    
    def get_torques(self, robot_state: ControlInput, max_torque: float):
        """Linearize dynamics about state point, state = [phi, phidot, thetadot]."""
        # Robot states vector
        state_vector = np.array(
            [
                robot_state.euler_angle_roll_rads,
                robot_state.euler_rate_roll_rads_s,
                robot_state.motor_speeds_roll_rads_s,
            ]
        )
        # Compute control input
        u = self.get_control(0, state_vector)
        # Return the computed torques

        out = torques(u[0], 0.0, 0.0)
        return out