import numpy as np

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque: float) -> np.array:
        """
        Calculates a torque using the optimal LQR gain matrix multiplied by the current robot state.
        If the calculated torque is greater than the specified maximum, a warning message will be logged
        and the torque will be clamped.

        Returns:
            torque: Desired torque for the LQR controller in [N*m] positive CW.
        """
        # Robot states vector
        state_vector = np.array(
            [
                robot_state.euler_angles_x_rad,
                robot_state.euler_angles_y_rad,
                robot_state.euler_angles_z_rad,
                robot_state.euler_rates_x_rad_s,
                robot_state.euler_rates_y_rad_s,
                robot_state.euler_rates_z_rad_s,
                robot_state.motor_speeds_pitch_rads_s,
                robot_state.motor_speeds_roll_rads_s,
                robot_state.motor_speeds_yaw_rads_s,
            ]
        )

        # Torque computation
        torques = np.clip(np.dot(self._K, state_vector), a_max= max_torque, a_min=-max_torque)

        return torques
