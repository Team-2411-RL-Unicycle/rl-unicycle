from collections import namedtuple

import numpy as np
import scipy.linalg as spla

from rluni.controller.fullrobot import LQRController, YawController
from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.controller.fullrobot.torque_filter import TorqueFilter
from rluni.utils.utils import call_super_first


class HighLevelXboxController(Controller):
    @call_super_first
    def __init__(self) -> None:
        self.logger.info(f"{self.__class__.__name__} initialized")
        self.yaw_controller = YawController(self.MAX_TORQUE_YAW)
        self.lqr_controller = LQRController()

    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError(
                "robot_state must be an instance of RobotState from the controller module"
            )
        return 0

    def handle_command(self, command: str, value):
        """
        Handle incoming commands (from MQTT, or some external pipeline)
        specific to the high-level logic for the Xbox controller.
        """
        if command == "xbox_button_a":
            # do something
            pass
        elif command == "xbox_stick_left":
            # do something
            pass
        else:
            # Optional: raise or log an unrecognized command
            pass
