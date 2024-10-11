import control as ct
import numpy as np
import sympy as sp

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class MPCController(Controller):

    @call_super_first
    def __init__(self) -> None:
        # Define symbolic variables
        mp, lp, Ip, mw, lw, Iw = sp.symbols('mp lp Ip mw lw Iw', real=True)
        phi, theta, dphi, dtheta = sp.symbols('phi theta dphi dtheta', real=True)
        tau = sp.symbols('tau', real=True)
        g = 9.81

        self.phi_sym = phi
        self.dphi_sym = dphi
        self.dtheta_sym = dtheta
        self.state_symbols = [phi, dphi, dtheta]

        # Parameters and values
        params = [mp, lp, Ip, mw, lw, Iw]
        values = [0.531, 0.100, 0.002250, 0.346, 0.180, 0.000725]

        # State variables
        q = sp.Matrix([phi, theta])
        dq = sp.Matrix([dphi, dtheta])

        # Potential energy mass
        m0 = (mp * lp + mw * lw) * g  # Effective U = mgh for combined parts

        # Mass matrix
        M = sp.Matrix([[Ip + mp * lp**2 + Iw + mw * lw**2, Iw],
                       [Iw, Iw]])

        # Lagrangian (kinetic energy - potential energy)
        lagrangian = 0.5 * dq.T * M * dq - m0 * sp.cos(phi)

        # Non-conservative forces (input torque)
        Q = sp.Matrix([0, tau])

        # Derive Euler-Lagrange equations
        eqs, ddq = self.euler_lagrange(q, dq, lagrangian, Q)

        # Solve for ddphi and ddtheta
        solutions = sp.solve(eqs, ddq)
        ddqArray = sp.Matrix([solutions[ddq[i]] for i in range(len(ddq))])

        # State vector and input
        X = sp.Matrix([phi, dphi, dtheta])
        U = sp.Matrix([tau])

        # State vector derivative
        dX = sp.Matrix([dphi, ddqArray[0], ddqArray[1]])

        # Symbolic Jacobian matrices A and B
        self._A_sym = dX.jacobian(X)
        self._B_sym = dX.jacobian(U)

        # Update with the physical parameters
        param_subs = dict(zip(params, values))
        self._A_sym = self._A_sym.subs(param_subs)
        self._B_sym = self._B_sym.subs(param_subs)

        # Define LQR weights
        self.phi_penalty = 10000
        self.phidot_penalty = 0.1
        self.thetadot_penalty = 0.015
        self.torque_penalty = 100

        # Store LQR weights
        self._Q = np.diag([self.phi_penalty, self.phidot_penalty, self.thetadot_penalty])
        self._R = np.array([[self.torque_penalty]])
        self.logger.info(f"{self.__class__.__name__} initialized")

    def euler_lagrange(self, q, dq, lagrangian, Q):
        dds = sp.Matrix([sp.symbols(f'dd{str(var)}') for var in dq])
        EQ = sp.zeros(len(q), 1)
        for i in range(len(q)):
            dL_dq = sp.diff(lagrangian, q[i])
            dL_ddq = sp.diff(lagrangian, dq[i])
            d_dt_dL_ddq = dL_ddq.jacobian(q.tolist() + dq.tolist()) @ sp.Matrix(q.tolist() + dds.tolist())
            EQ[i] = sp.simplify(Q[i] - d_dt_dL_ddq + dL_dq)
        return EQ, dds

    def compute_lqr_gain(self, X_current):
        """
        Compute the LQR gain matrix K at the current state X_current

        Inputs:
            - X_current: state vector of the robot holding
                - [phi, dphi, dtheta]
        """
        state_subs = {
            self.phi_sym: X_current[0],
            self.dphi_sym: X_current[1],
            self.dtheta_sym: X_current[2],
        }

        # Convert symbolic matrices to numerical numpy arrays
        A_num = np.array(self._A_sym.subs(state_subs).evalf(), dtype=np.float64)
        B_num = np.array(self._B_sym.subs(state_subs).evalf(), dtype=np.float64)
        
        K, S, E = ct.lqr(A_num, B_num, self._Q, self._R)
        
        return K

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the optimal LQR gain matrix multiplied by the current robot state.
        If the calculated torque is greater than the specified maximum, a warning message will be logged
        and the torque will be clamped.

        Returns:
            torque: Desired torque for the MPC controller.
        """
        # Robot states vector
        state_vector = np.array(
            [
                robot_state.pendulum_angle,
                robot_state.pendulum_vel,
                robot_state.wheel_vel,
            ]
        )

        K = self.compute_lqr_gain(state_vector)

        # Torque computation
        torque = -np.dot(K, state_vector)

        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque
