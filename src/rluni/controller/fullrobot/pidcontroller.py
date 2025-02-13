import importlib.resources as pkg_resources

import numpy as np
from simple_pid import PID

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils import call_super_first
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file


class PIDController(Controller):

    @call_super_first
    def __init__(self, config_file=None) -> None:
        # Load configuration from YAML file

        config_file = config_file or pkg_resources.resource_filename(
            "rluni.config.controllers", "pid_projectfair.yaml"
        )
        self.config = load_config_file(config_file)

        # Initialize parameters from the config
        self._Kp = gvcv(self.config, "PIDController.Kp", float, required=True)
        self._Ki = gvcv(self.config, "PIDController.Ki", float, required=True)
        self._Kd = gvcv(self.config, "PIDController.Kd", float, required=True)

        self._Kp_wheel_vel = gvcv(
            self.config, "PIDController.Kp_wheel_vel", float, required=True
        )
        self._Ki_wheel_vel = gvcv(
            self.config, "PIDController.Ki_wheel_vel", float, required=True
        )
        self._Kd_wheel_vel = gvcv(
            self.config, "PIDController.Kd_wheel_vel", float, required=True
        )

        self._max_del_s = gvcv(
            self.config, "PIDController.max_del_s", float, required=True
        )

        self.downsample = gvcv(
            self.config, "PIDController.downsample", int, required=True
        )

        # Initialize PID controllers
        self._pid = PID(self._Kp, self._Ki, self._Kd, setpoint=0)
        self._pid_wheel = PID(self._Kp_wheel_vel, 0, 0, setpoint=0)

        self.downsample_counter = 0
        self.buffer = [0] * self.downsample

        self.logger.info(
            f"{self.__class__.__name__} initialized with parameters from config."
        )

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the PID error of pendulum angle. If the calculated torque
        is greater than the specified maximum, a warning message will be logged and the torque
        will be clamped.

        Returns:
            torque: Desired torque for the PID controller.
        """
        # Update control setpoint based on wheel velocity
        self.update_control_setpoint(-robot_state.wheel_vel / (2 * np.pi))
        # Calculate torque
        torque = self._pid(robot_state.pendulum_angle * 180 / np.pi)
        # Clamp torque if outside bounds
        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0 else -1)

        return -torque

    def update_control_setpoint(self, wheel_vel: float) -> float:
        """
        Calculate the setpoint for the PID controller based on the wheel velocity.

        Theory: create a saturating P controller in response to deviation from 0 velocity on wheel
        The controller saturates at max_del_s degrees of pendulum angle.
        """
        self.downsample_counter = (self.downsample_counter + 1) % self.downsample
        self.buffer[self.downsample_counter] = wheel_vel
        if self.downsample_counter == 0:
            wheel_vel = np.mean(self.buffer[-3])
            setpoint = -self._pid_wheel(wheel_vel)
            self._pid.setpoint = np.clip(setpoint, -self._max_del_s, self._max_del_s)

    def update_parameter(self, param: str, value: float):
        """
        Update PID Controller's control parameter to a new value.

        Args:
            param: must be 'P', 'I', or 'D'
            value: value of parameter
        """
        if param == "P":
            self._Kp_wheel_vel = value
        elif param == "I":
            self._Ki_wheel_vel = value
        elif param == "D":
            self._Kd_wheel_vel = value
        # Raise error if not P, I, or D
        else:
            raise ValueError(f"Invalid parameter: {param}")

        self._pid_wheel.reset()
        self._pid_wheel.tunings = (
            self._Kp_wheel_vel,
            self._Ki_wheel_vel,
            self._Kd_wheel_vel,
        )
        return
