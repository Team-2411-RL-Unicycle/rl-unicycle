
import numpy as np
import importlib.resources as pkg_resources
import sympy as sp
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import scipy.linalg
import cvxpy as cp
import time as clock

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils import call_super_first
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

class MPCController(Controller):
    def __init__(
        self, A, B, Q, R, Qf=None, N=20, tau_max=1.0, warm_start=True, solver_kwargs={}
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
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.Qf = Qf if Qf is not None else Q
        self.tau_max = tau_max
        self.N = N

        self.n = A.shape[0]
        self.m = B.shape[1]

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
                == self.A @ self.x_var[:, k] + self.B @ self.u_var[:, k],
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
        x = x[[0, 2, 3]]  # pare down to 3 states of interest
        return self.solve_mpc(x, warm_start=True)