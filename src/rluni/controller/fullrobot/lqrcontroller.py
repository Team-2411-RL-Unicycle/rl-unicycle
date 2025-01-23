from collections import namedtuple

import numpy as np
import scipy.linalg as spla

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array(
            [
                [-42.1348*1, 0.0, 0.0, -2.7488/32, 0.0, 0.0, 0.0032*10, 0.0, 0.0],  # roll
                [0.0, -10.6087/3, 0.0, 0.0, -2.5392/10, 0.0, 0.0, 0.0039, 0.0],  # pitch
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # yaw
            ]
        )
        # Q=np.diag([
        #     10000, 1, 1/1000,  # roll
        #     10000/100, 1*10, 1/1000,  # pitch
        #     0, 0, 0
        # ])
        # R=np.diag([100, 100*100, 100])
        # self._K = self.compute_K_mat(Q,R)
        self.logger.info(f"{self.__class__.__name__} initialized")

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

        scale = 1.0
        # Torque computation
        # out = scale * np.dot(self._K, state_vector)
        out = scale * self._K @ state_vector

        # torques = np.clip(
        #     scale * np.dot(self._K, state_vector), a_max=max_torque, a_min=-max_torque
        # )

        # DEBUG


        for i, component in enumerate(["roll"]):#, "pitch", "yaw"]):
            row_contribution = self._K[i] * state_vector  # Element-wise contribution
            print(f"{component.capitalize()} torque breakdown:")
            for j, value in enumerate(row_contribution):
                print(f"  State {j}: Gain {self._K[i, j]:.3f} * State {state_vector[j]:.3f} = {value:.3f}")
            print(f"  Total {component} torque before scaling: {out[i] / 0.1:.3f}")
            print(f"  Total {component} torque after scaling: {out[i]:.3f}\n")


        # needs clipping
        torques = torques(
            np.clip(out[0], a_min=-1.4, a_max=1.4),  # roll
            np.clip(out[1], a_min=-1.0, a_max=1.0),  # pitch
            # temporarily clip to 0.1
            # np.clip(out[0], a_min=-0.47, a_max=0.47),  # roll
            # np.clip(out[1], a_min=-0.47, a_max=0.47),  # pitch
            np.clip(out[2], a_min=-0.17, a_max=0.17),  # yaw
        )

        return torques
    
    def compute_K_mat(self, Q, R):
        A = np.array([
            [0, 1, 0, 0, 0, 0],       # Row 1 (Roll dynamics)
            [36.5724, 0, 0, 0, 0, 0], # Row 2 (Roll dynamics)
            [-36.5724, 0, 0, 0, 0, 0],# Row 3 (Roll dynamics)
            [0, 0, 0, 0, 1, 0],       # Row 4 (Pitch dynamics)
            [0, 0, 0, 16.6435, 0, 0], # Row 5 (Pitch dynamics)
            [0, 0, 0, -1.2029, 0, 0]  # Row 6 (Pitch dynamics)
        ])

        A_full = np.zeros((9,9))
        A_full[0:6,0:6]=A
        A_full[6:9,6:9] = np.eye(3)

        B = np.array([
            [0, 0],                   # Row 1 (Roll input)
            [-14.2, 0],               # Row 2 (Roll input)
            [1316.3, 0],              # Row 3 (Roll input)
            [0, 0],                   # Row 4 (Pitch input)
            [0, -3.2179],             # Row 5 (Pitch input)
            [0, 17.6336]              # Row 6 (Pitch input)
        ])
        B_full = np.zeros((9,3))
        B_full[0:6,0:2]=B

        P = spla.solve_continuous_are(A_full, B_full, Q, R)
        K = np.linalg.inv(R) @ B_full.T @ P
        return K
