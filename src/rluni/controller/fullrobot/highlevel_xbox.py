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
        self.yaw_controller = YawController(self.MAX_TORQUE_YAW)

    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError(
                "robot_state must be an instance of RobotState from the controller module"
            )
        return 0