from collections import namedtuple

import numpy as np
import scipy.linalg as spla

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.controller.fullrobot.torque_filter import TorqueFilter
from rluni.utils.utils import call_super_first

DEG_TO_RAD = np.pi / 180


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        # stable feb 5 w/ .75-.85 ema
        # Q_roll = np.diag([1e6, 5, 16e-1])
        # R_roll = np.diag([8e4])

        Q_roll = np.diag([1e6, 5, 16e-1])
        R_roll = np.diag([8e4])
        Q_pitch = np.diag([1e-6, 1e-1, 1e5])
        R_pitch = np.diag([1e12])
        self._K = self.compute_LQR_gain(Q_roll, R_roll, Q_pitch, R_pitch)
        self.torque_filter = TorqueFilter()

        # Temporary overide
        self._K[0, 0], self._K[0, 3], self._K[0, 6] = (
            self._K[0, 0] * 0.75,
            self._K[0, 3] * 0.65,
            self._K[0, 6] * 1.0,
        )
        self._K[1, 1], self._K[1, 4], self._K[1, 7] = -10.6 / 3, -2.5392 / 10, 0.0059

        self.logger.info(f"{self.__class__.__name__} initialized")

    def compute_LQR_gain(self, Q_roll, R_roll, Q_pitch, R_pitch):
        A = np.array(
            [
                [0, 1, 0, 0, 0, 0],  # Row 1 (Roll dynamics)
                [36.5724, 0, 0, 0, 0, 0],  # Row 2 (Roll dynamics)
                [-36.5724, 0, 0, 0, 0, 0],  # Row 3 (Roll dynamics)
                [0, 0, 0, 0, 1, 0],  # Row 4 (Pitch dynamics)
                [0, 0, 0, 16.6435, 0, 0],  # Row 5 (Pitch dynamics)
                [0, 0, 0, -1.2029, 0, 0],  # Row 6 (Pitch dynamics)
            ]
        )

        B = np.array(
            [
                [0, 0],  # Row 1 (Roll input)
                [-14.2, 0],  # Row 2 (Roll input)
                [1316.3, 0],  # Row 3 (Roll input)
                [0, 0],  # Row 4 (Pitch input)
                [0, -3.2179],  # Row 5 (Pitch input)
                [0, 17.6336],  # Row 6 (Pitch input)
            ]
        )

        # Create the block diagonal matrix for Q
        Q = spla.block_diag(Q_roll, Q_pitch)

        # Create the block diagonal matrix for R
        R = spla.block_diag(R_roll, R_pitch)

        # Solve Riccati equation
        P = spla.solve_continuous_are(A, B, Q, R)

        # Compute LQR gain
        K = np.linalg.inv(R) @ B.T @ P
        # Trim close to zero values
        K = np.where(np.abs(K) < 1e-10, 0, K)
        K_roll = K[0, :3]
        K_pitch = K[1, -3:]
        print(K_roll, K_pitch)
        # Full K construction
        K_full = np.zeros((9, 9))
        K_full[0, 0], K_full[0, 3], K_full[0, 6] = K_roll[0], K_roll[1], K_roll[2]
        K_full[1, 1], K_full[1, 4], K_full[1, 7] = K_pitch[0], K_pitch[1], K_pitch[2]

        return K_full

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque: float) -> np.array:
        """
        Calculates the torques using the optimal LQR gain matrix multiplied by the current robot state.
        If the calculated torque is greater than the specified maximum, a warning message will be logged
        and the torque will be clamped.

        Returns:
            torques (Roll, Pitch, Yaw): Desired torque for the LQR controller in [N*m] positive CW.
        """
        torques = namedtuple("torques", ["roll", "pitch", "yaw"])
        # Robot states vector
        state_vector = np.array(
            [
                robot_state.euler_angle_roll_rads,
                robot_state.euler_angle_pitch_rads,
                robot_state.euler_angle_yaw_rads,
                robot_state.euler_rate_roll_rads_s,
                robot_state.euler_rate_pitch_rads_s,
                robot_state.euler_rate_yaw_rads_s,
                robot_state.motor_speeds_roll_rads_s,
                robot_state.motor_speeds_pitch_rads_s
                + robot_state.euler_rate_pitch_rads_s,  # accounting for robot rotation
                robot_state.motor_speeds_yaw_rads_s,
            ]
        )

        # state_vector[1] += self.pitch_drift_correction(
        #     angle_limit=2.0 * DEG_TO_RAD,
        #     motor_position=robot_state.motor_position_pitch_rads,
        # )

        scale = 1.0
        out = scale * self._K @ state_vector

        # DEBUG
        # for i, component in enumerate(["roll", "pitch", "yaw"]):
        #     row_contribution = self._K[i] * state_vector  # Element-wise contribution
        #     print(f"{component.capitalize()} torque breakdown:")
        #     for j, value in enumerate(row_contribution):
        #         print(
        #             f"  State {j}: Gain {self._K[i, j]:.3f} * State {state_vector[j]:.3f} = {value:.3f}"
        #         )
        #     print(f"  Total {component} torque before scaling: {out[i] / 0.1:.3f}")
        #     print(f"  Total {component} torque after scaling: {out[i]:.3f}\n")

        # Trim the roll torque
        roll_t = np.clip(out[0], a_min=-1.4, a_max=1.4)
        roll_t = np.sign(roll_t) * np.abs(roll_t) ** 1.1

        # TODO conditioning on the torque
        # # Apply filter only to the roll torque
        # filtered_roll = self.torque_filter.process_torque(roll_t)

        torques = torques(
            roll_t,  # roll
            np.clip(out[1], a_min=-1.0, a_max=1.0),  # pitch
            np.clip(out[2], a_min=-0.17, a_max=0.17),  # yaw
        )

        return torques

    def compute_K_mat(self, Q, R) -> np.array:
        """
        Computes the LQR gain matrix (K) using the state weighting matrix (Q) and
        input weighting matrix (R), returning a 9x9 matrix for roll, pitch, and yaw control.

        Args:
            Q (np.array): State weighting matrix for state error penalties.
            R (np.array): Input weighting matrix for control effort penalties.

        Returns:
            K_full (np.array): A 9x9 LQR gain matrix combining roll, pitch, and yaw dynamics.
        """
        A = np.array(
            [
                [0, 1, 0, 0, 0, 0],  # Row 1 (Roll dynamics)
                [36.5724, 0, 0, 0, 0, 0],  # Row 2 (Roll dynamics)
                [-36.5724, 0, 0, 0, 0, 0],  # Row 3 (Roll dynamics)
                [0, 0, 0, 0, 1, 0],  # Row 4 (Pitch dynamics)
                [0, 0, 0, 16.6435, 0, 0],  # Row 5 (Pitch dynamics)
                [0, 0, 0, -1.2029, 0, 0],  # Row 6 (Pitch dynamics)
            ]
        )

        B = np.array(
            [
                [0, 0],  # Row 1 (Roll input)
                [-14.2, 0],  # Row 2 (Roll input)
                [1316.3, 0],  # Row 3 (Roll input)
                [0, 0],  # Row 4 (Pitch input)
                [0, -3.2179],  # Row 5 (Pitch input)
                [0, 17.6336],  # Row 6 (Pitch input)
            ]
        )

        P = spla.solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P

        # sub roll and pitch mat into a full roll, pitch, and yaw mat
        K_full = np.zeros((9, 9))
        K_full[0, 0], K_full[0, 3], K_full[0, 6] = K[0, 0], K[0, 1], K[0, 2]
        K_full[1, 1], K_full[1, 4], K_full[1, 7] = K[1, 3], K[1, 4], K[1, 5]

        return K_full

    def pitch_drift_correction(
        self, angle_limit: float, motor_position: float
    ) -> float:
        """
        Corrects the pitch angle drift by adding a bias on either side of a local point.

        Args:
            angle_limit (float): The maximum bias in radians.
            motor_position (float): The current angle offset in radians.

        Returns:
            angle_offset_clipped (float): The bias to add to pitch angle.
        """
        print("\n\n******")
        angle_offset = motor_position % (2 * np.pi)
        print("local", angle_offset)
        angle_offset = (
            angle_offset - (2 * np.pi) if angle_offset > np.pi else angle_offset
        )
        print("centered", angle_offset)
        angle_offset = -angle_offset * angle_limit / np.pi
        print("scaled", angle_offset)
        angle_offset_clipped = np.clip(
            angle_offset, a_min=-angle_limit, a_max=angle_limit
        )
        print("clipped", angle_offset_clipped)
        return angle_offset_clipped
