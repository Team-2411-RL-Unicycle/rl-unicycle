import numpy as np

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first

DEG_TO_RAD = np.pi / 180
REV_TO_RAD = 2 * np.pi


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array([13.1127, 1.1050, 0.0122])
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
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
                robot_state.pendulum_angle * DEG_TO_RAD,  # radians
                robot_state.pendulum_vel * DEG_TO_RAD,  # radians/s
                -robot_state.wheel_vel * REV_TO_RAD,  # radians/s
            ]
        )

        # Torque computation
        torque = -np.dot(self._K, state_vector)

        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque
