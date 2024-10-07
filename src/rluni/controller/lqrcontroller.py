import numpy as np

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = np.array([0.35, 0.02, -0.07, 0.5])
        self.state_pend_angle = 0
        self.state_pend_vel = 1
        self.state_wheel_vel = 2
        self.state_roll_torque = 3
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the optimal LQR gain matrix multiplied by the current robot state.
        If the calculated torque is greater than the specified maximum, a warning message will be logged
        and the torque will be clamped.

        Returns:
            torque: Desired torque for the LQR controller.
        """
        # Robot states vector
        state_vector = np.array([
            robot_state.pendulum_angle,
            robot_state.pendulum_vel,
            robot_state.wheel_vel,
            robot_state.roll_torque
        ])

        # Torque computation
        torque = -np.dot(self._K, state_vector)

        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque
