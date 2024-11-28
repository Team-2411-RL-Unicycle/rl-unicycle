import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array(
            [
                [21.8516, 0.0, 0.0, 3.2252, 0.0, 0.0, -0.0122, 0.0, 0.0],  # roll
                [0.0, 20.9947, 0.0, 0.0, 4.5424, 0.0, 0.0, -0.0122, 0.0],  # pitch
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )  # yaw
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
                robot_state.motor_speeds_pitch_rads_s,
                robot_state.motor_speeds_yaw_rads_s,
            ]
        )

        # Torque computation
        torques = np.clip(
            0.1 * np.dot(self._K, state_vector), a_max=max_torque, a_min=-max_torque
        )

        return torques
