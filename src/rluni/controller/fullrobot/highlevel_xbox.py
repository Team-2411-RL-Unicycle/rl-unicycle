from collections import namedtuple

import numpy as np
import scipy.linalg as spla

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.controller.fullrobot.torque_filter import TorqueFilter
from rluni.controller.fullrobot import YawController, LQRController
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils.utils import call_super_first, load_config_file
from importlib.resources import files


class HighLevelXboxController(Controller):
    @call_super_first
    def __init__(self) -> None:
        self.logger.info(f"{self.__class__.__name__} initialized")
        self.MAX_TORQUE_YAW = 0.04
        self.yaw_controller = YawController(self.MAX_TORQUE_YAW)
        self.lqr_controller = LQRController()
        self.current_pitch = 0.0
        config = self._load_config("unicycle.yaml")
        self.PITCH_RADIUS = gvcv(
            config, "RobotSystem.pitch_wheel_radius", float, required=True
        )

        self.current_roll = 0.0

        self.current_yaw = 0.0

    def _update_pitch(self, pitch: float):
        """Clamp pitch to [-1, 1] and update the current pitch."""
        if pitch < -1.0:
            self.logger.warning(f"Pitch value {pitch} is below -1.0. Clamping to -1.0.")
            pitch = -1.0
        elif pitch > 1.0:
            self.logger.warning(f"Pitch value {pitch} is above 1.0. Clamping to 1.0.")
            pitch = 1.0

        # Bias on the positive pitch direction
        if pitch > 0:
            pitch = pitch * 1.1

        self.current_pitch = 3 * pitch * np.pi / 180

    def _update_roll(self, roll: float):
        """Clamp roll to [-1, 1] and update the current roll."""
        if roll < -1.0:
            self.logger.warning(f"Roll value {roll} is below -1.0. Clamping to -1.0.")
            roll = -1.0
        elif roll > 1.0:
            self.logger.warning(f"Roll value {roll} is above 1.0. Clamping to 1.0.")
            roll = 1.0

        self.current_roll = -3 * roll * np.pi / 180

    def _update_yaw(self, yaw: float):
        """Clamp yaw to [-1, 1] and update the current yaw."""
        if yaw < -1.0:
            self.logger.warning(f"Yaw value {yaw} is below -1.0. Clamping to -1.0.")
            yaw = -1.0
        elif yaw > 1.0:
            self.logger.warning(f"Yaw value {yaw} is above 1.0. Clamping to 1.0.")
            yaw = 1.0

        self.current_yaw = yaw

    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        # Update the pitch target based on the current pitch value

        robot_state.euler_angle_pitch_rads -= self.current_pitch
        robot_state.euler_angle_roll_rads -= self.current_roll

        roll, pitch, yaw = self.lqr_controller.get_torques(robot_state, max_torque)
        yaw = self.yaw_controller.get_torques(self.current_yaw)

        torques = namedtuple("torques", ["roll", "pitch", "yaw"])
        return torques(roll, pitch, yaw)

    def handle_command(self, command: str, value):
        """
        Handle incoming commands (from MQTT, or some external pipeline)
        specific to the high-level logic for the Xbox controller.
        """
        if command == "pitch":
            self._update_pitch(value)

        if command == "roll":
            self._update_roll(value)

        if command == "yaw":
            self._update_yaw(value)

        elif command == "xbox_stick_left":
            # do something
            pass
        else:
            # Optional: raise or log an unrecognized command
            pass
        
    def _load_config(self, config_file):
        """Load the robot configuration file and set relevant parameters."""
        # Load the robot configuration file (use the default if none provided)
        if config_file is None:
            config_file = "unicycle.yaml"
        
        config_file_path = files("rluni.configs.robot").joinpath(config_file)
        
        # Load the configuration file
        config = load_config_file(config_file_path)
        return config
