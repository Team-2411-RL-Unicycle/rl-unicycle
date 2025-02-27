from collections import namedtuple

import numpy as np
import scipy.linalg as spla

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.controller.fullrobot.torque_filter import TorqueFilter
from rluni.controller.fullrobot import YawController, LQRController
from rluni.utils.utils import call_super_first


class HighLevelXboxController(Controller):
    @call_super_first
    def __init__(self) -> None:
        self.logger.info(f"{self.__class__.__name__} initialized")
        self.MAX_TORQUE_YAW = 0.1
        self.yaw_controller = YawController(self.MAX_TORQUE_YAW)
        self.lqr_controller = LQRController()
        self.current_pitch = 0.0
        self.current_target_pitch_rad_s = 0.0
        self.PITCH_WHEEL_RADIUS = 0.055
        

    def _update_pitch(self, pitch: float):
        """Clamp pitch to [-1, 1] and update the current pitch."""
        if pitch < -1.0:
            self.logger.warning(f"Pitch value {pitch} is below -1.0. Clamping to -1.0.")
            pitch = -1.0
        elif pitch > 1.0:
            self.logger.warning(f"Pitch value {pitch} is above 1.0. Clamping to 1.0.")
            pitch = 1.0

        self.current_pitch = pitch
        self.current_target_pitch_rad_s = self._pitch_to_rotational_velocity_target(pitch)

    def _pitch_to_rotational_velocity_target(self, pitch: float) -> float:
        # Function to map [-1,1] to a linear velocity target
        def f(x):
            return 2.0 * x  # per 1 unit of pitch

        target_vel = f(pitch)

        # convert linear velocity to rotational velocity
        target_rotational_vel = target_vel / self.PITCH_WHEEL_RADIUS  # rad/s
        return target_rotational_vel

    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        # Update the pitch target based on the current pitch value

        robot_state.motor_speeds_pitch_rads_s -= self.current_target_pitch_rad_s
        return self.lqr_controller.get_torques(robot_state, max_torque)

    def handle_command(self, command: str, value):
        """
        Handle incoming commands (from MQTT, or some external pipeline)
        specific to the high-level logic for the Xbox controller.
        """
        if command == "pitch":
            self._update_pitch(value)

        elif command == "xbox_stick_left":
            # do something
            pass
        else:
            # Optional: raise or log an unrecognized command
            pass
