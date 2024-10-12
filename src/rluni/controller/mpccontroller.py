import control as ct
import numpy as np
import sympy as sp

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class MPCController(Controller):

    @call_super_first
    def __init__(self) -> None:
        # Define symbolic variables
        mp, lp, Ip, mw, lw, Iw = sp.symbols("mp lp Ip mw lw Iw", real=True)
        phi, theta, dphi, dtheta = sp.symbols("phi theta dphi dtheta", real=True)
        tau = sp.symbols("tau", real=True)
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
        M = sp.Matrix([[Ip + mp * lp**2 + Iw + mw * lw**2, Iw], [Iw, Iw]])

        # Lagrangian (kinetic energy - potential energy)
        lagrangian = 0.5 * (dq.T * M * dq)[0] - m0 * sp.cos(phi)

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
        self._Q = np.diag(
            [self.phi_penalty, self.phidot_penalty, self.thetadot_penalty]
        )
        self._R = np.array([[self.torque_penalty]])
        self.logger.info(f"{self.__class__.__name__} initialized")

    def euler_lagrange(self, q, dq, lagrangian, Q, verbosity=0):
        """
        Compute the Euler-Lagrange equations of motion for a given system.

        The Euler-Lagrange equation describes the dynamics of a system, where the
        generalized coordinates (state variables) and their derivatives are used to
        compute the forces acting on the system.

        Equation:
            d   dL       dL
            -- ----  -  ---- = Q
            dt d(dq)     dq

        Parameters:
            q : list or sympy Matrix
                The state vector (generalized coordinates), represented as symbols.
            dq : list or sympy Matrix
                The time derivatives of the state vector (generalized velocities),
                represented as symbols.
            lagrangian : sympy expression
                The Lagrangian of the system, defined as the difference between kinetic
                and potential energy (L = T - V).
            Q : list or sympy Matrix
                The generalized forces acting on the system (non-conservative forces).
            verbosity : int, optional
                Integer controlling the level of verbosity:
                0 - print nothing (default)
                1 - print result
                2 - print result and derivative terms

        Returns:
            EQ : sympy Matrix
                The solved equations of motion for the system (accelerations).
            dds : sympy Matrix
                The symbolic variables representing the second derivatives (accelerations).
        """
        q = sp.Matrix(q)
        dq = sp.Matrix(dq)
        Q = sp.Matrix(Q)

        if len(q) != len(dq):
            print(f"[Euler-Lagrange Error] q and dq must have equal lengths! (len(q) = {len(q)}, len(dq) = {len(dq)})")
            return None, None
        elif len(q) != len(Q):
            print(f"[Euler-Lagrange Error] q and Q must have equal lengths! (len(q) = {len(q)}, len(Q) = {len(Q)})")
            return None, None

        dds = sp.Matrix([sp.symbols(f"d{str(dq_i)}") for dq_i in dq])


        if verbosity > 1:
            print("Noting higher derivatives as:")
            print(f"dds = [ {' '.join(map(str, dds))} ]\n")

        EQ = sp.zeros(len(q), 1)
        for i in range(len(q)):
            # Derivative of Lagrangian w.r.t. q and dq
            partial_q = sp.diff(lagrangian, q[i])
            partial_dq = sp.diff(lagrangian, dq[i])

            partial_dq_matrix = sp.Matrix([partial_dq])
            variables = sp.Matrix.vstack(q, dq)
            derivatives = sp.Matrix.vstack(dq, dds)
            jacobian_matrix = partial_dq_matrix.jacobian(variables)
            partial_dt_dq_matrix = jacobian_matrix * derivatives
            partial_dt_dq = partial_dt_dq_matrix[0]

            # Solve Euler-Lagrange equation with input forces, Q
            eq = sp.Eq(Q[i], partial_dt_dq - partial_q)
            solution = sp.solve(eq, dds[i], dict=True)

            if solution:
                # If solution exists, extract the solution for dds[i]
                EQ[i] = self.reduce(solution[0][dds[i]])
            else:
                print(f"[Euler-Lagrange Warning] State {i+1} did not have a non-zero solution.")
                EQ[i] = sp.sympify(0)

            # Verbosity outputs
            if verbosity > 0:
                if verbosity > 1:
                    print("------------------------------------")
                    print(f"Derivative of the potential term for state {i+1}:")
                    print(f"partial_q = {self.reduce(partial_q)}\n")
                    print("------------------------------------")
                    print(f"Derivative of the kinetic term for state {i+1}:")
                    print(f"partial_dt_dq = {self.reduce(partial_dt_dq)}\n")
                print("------------------------------------")
                print(f"General Equation for state {i+1}:")
                print(f"{dds[i]} = {EQ[i]}\n")

        return EQ, dds

    def reduce(self, expr):
        """Utility function to simplify expressions."""
        return sp.simplify(sp.expand(expr))

    def compute_lqr_gain(self, X_current):
        """
        Compute the Linear Quadratic Regulator (LQR) gain matrix K for the current state.

        The LQR gain matrix K is computed by linearizing the system dynamics around
        the current state and solving the continuous-time algebraic Riccati equation.

        Parameters:
            X_current : The current state of the system, a vector of the form [phi, dphi, dtheta].
                - phi: Current angle of the pendulum from vertical (radians). Positive CCW
                - dphi: Current angular velocity of the pendulum (radians/second). Positive CCW
                - dtheta: Current angular velocity of the wheel (radians/second). Positive CW

        Returns:
            K : The LQR gain matrix. Used to compute the control input based on the current
                state of the system.
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
        Apply the LQR control law to compute the control input (torque) based on the current state.

        Parameters:

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
        torque = np.dot(K, state_vector)

        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque
