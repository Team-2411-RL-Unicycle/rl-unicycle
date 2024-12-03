from collections import namedtuple

import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array(
            [
                [-14.5702, 0.0, 0.0, -1.7557/10, 0.0, 0.0, 0.015, 0.0, 0.0],  # roll
                [0.0, -10.6087/3, 0.0, 0.0, -2.5392/10, 0.0, 0.0, 0.0039, 0.0],  # pitch
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # yaw
            ]
        )
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
        print(f"Roll total torque: {out[0]}")
        print(f"% Roll angle: {scale*state_vector[0]*self._K[0][0]/out[0]}")
        print(f"% Roll angular vel: {scale*state_vector[3]*self._K[0][3]/out[0]}")
        print(f"% Roll wheel spd: {scale*state_vector[6]*self._K[0][6]/out[0]}")
        print()
        print(f"Pitch total torque: {out[1]}")
        print(f"% Pitch angle: {scale*state_vector[1]*self._K[1][1]/out[1]}")
        print(f"% Pitch angular vel: {scale*state_vector[4]*self._K[1][4]/out[1]}")
        print(f"% Pitch wheel spd: {scale*state_vector[7]*self._K[1][7]/out[1]}")
        print()

        # print()


        # for i, component in enumerate(["roll", "pitch", "yaw"]):
        #     row_contribution = self._K[i] * state_vector  # Element-wise contribution
        #     print(f"{component.capitalize()} torque breakdown:")
        #     for j, value in enumerate(row_contribution):
        #         print(f"  State {j}: Gain {self._K[i, j]:.3f} * State {state_vector[j]:.3f} = {value:.3f}")
        #     print(f"  Total {component} torque before scaling: {out[i] / 0.1:.3f}")
        #     print(f"  Total {component} torque after scaling: {out[i]:.3f}\n")


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
