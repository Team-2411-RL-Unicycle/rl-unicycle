import control as ct
import numpy as np

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class MPCController(Controller):

    @call_super_first
    def __init__(self, method) -> None:

        # method of computing lqr (set to None by default)
        self.method = method

        # Define LQR weights
        self.phi_penalty = 10000
        self.phidot_penalty = 0.1
        self.thetadot_penalty = 0.015
        self.constant_penalty = 0
        self.torque_penalty = 100

        # Store LQR weights
        self._Q = np.diag(
            [
                self.phi_penalty,
                self.phidot_penalty,
                self.thetadot_penalty,
                self.constant_penalty,
            ]
        )
        self._R = np.array([[self.torque_penalty]])

    def augmented_system_A(self, phi: float) -> np.ndarray:
        """Compute and return the system's augmented A matrix, evaluated numerically."""
        # cos and sin phi
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)

        # eval 4x4 matrix numerically
        A = np.array(
            [
                [0, 1, 0, 0],
                [60.301 * cos_phi, 0, 0, 60.301 * sin_phi],
                [60.301 * cos_phi, 0, 0, 60.301 * sin_phi],
                [0, 0, 0, 1],
            ]
        )

        return A

    def augmented_system_B(self) -> np.ndarray:
        """Return the system's augmented input array B."""
        B = np.array([[0], [53.275], [1432.6], [1]])

        return B

    def compute_lqr_gain(self, X_current: np.ndarray) -> np.ndarray:
        """
        Calculate the LQR gain matrix K for the current system state.

        Parameters:
            X_current (np.ndarray): Current state vector [phi, dphi, dtheta]
                - phi (float): Pendulum angle from vertical (radians), positive CCW
                - dphi (float): Pendulum angular velocity (rad/s), positive CCW
                - dtheta (float): Wheel angular velocity (rad/s), positive CW

        Returns:
            np.ndarray: LQR gain matrix K, used to compute control input.
        """
        phi = X_current[0]

        # Convert symbolic matrices to numerical numpy arrays
        A_num = self.augmented_system_A(phi=phi)
        B_num = self.augmented_system_B()

        K, S, E = ct.lqr(A_num, B_num, self._Q, self._R, method=self.method)

        return K.flatten()[:3]

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
