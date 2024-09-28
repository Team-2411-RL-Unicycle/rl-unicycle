import numpy as np

from rluni.controller.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class LQRController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self._K = [97.2590,2.1734,23.1089]
        self._max_rps = 35  # revs/s
        self._max_del_s = 2.5  # degrees
        self.state_pend_angle = 0
        self.state_pend_vel = 1
        self.state_wheel_vel = 2
        self.logger.info(f"{self.__class__.__name__} initialized")
        self.downsample = 25
        self.downsample_counter = 0
        self.buffer = [0] * self.downsample

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the optimal LQR gain matrix multiplied by the error of pendulum angle. 
        If the calculated torque is greater than the specified maximum, a warning message will be logged 
        and the torque will be clamped.

        Returns:
            torque: Desired torque for the LQR controller.
        """
        # Calculate torque
        torque = -0.1*(
            self._K[self.state_pend_angle]*robot_state.pendulum_angle + 
            self._K[self.state_pend_vel]*robot_state.pendulum_vel + 
            self._K[self.state_wheel_vel]*robot_state.wheel_vel
        )

        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque
