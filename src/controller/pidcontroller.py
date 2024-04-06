from controller.controllerABC import Controller, ControlInput
from simple_pid import PID
import numpy as np


class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self._Kp = 0.07
        self._Ki = 0.00
        self._Kd = 0.0045
        self._Kp_wheel_vel = -0.11  # 0.1 - 0.2 is a potential value range to check
        self._max_rps = 35  # revs/s
        self._max_del_s = 3  # degrees
        self._pid = PID(self._Kp, self._Ki, self._Kd, setpoint=0)
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the PID error of pendulum angle. If the calculated torque
        is greater than the specified maximum, a warning message will be logged and the torque
        will be clamped.

        Returns:
            torque: Desired torque for the PID controller.
        """
        super().get_torque(robot_state, max_torque)
        # Update control setpoint based on wheel velocity
        self.update_control_setpoint(robot_state.wheel_vel)
        # Calculate torque
        torque = self._pid(robot_state.pendulum_angle)
        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return torque

    def update_control_setpoint(self, wheel_vel: float) -> float:
        """
        Calculate the setpoint for the PID controller based on the wheel velocity.

        Theory: create a saturating P controller in response to deviation from 0 velocity on wheel
        The controller saturates at max_del_s degrees of pendulum angle.
        """
        setpoint = wheel_vel * self._Kp_wheel_vel
        self._pid.setpoint = np.clip(setpoint, -self._max_del_s, self._max_del_s)

    def update_parameter(self, param: str, value: float):
        """
        Update PID Controller's control parameter to a new value.

        Args:
            param: must be 'P', 'I', or 'D'
            value: value of parameter
        """
        if param == "P":
            self._Kp = value
        elif param == "I":
            self._Ki = value
        elif param == "D":
            self._Kd = value
        # Raise error if not P, I, or D
        else:
            raise ValueError(f"Invalid parameter: {param}")

        self._pid.reset()
        self._pid.tunings = (self._Kp, self._Ki, self._Kd)
        return
