import math
from collections import namedtuple

import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class TestController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.gen = self.sine_sequence()
        self.MAX_TORQUE = 0.1
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torques(self, robot_state: ControlInput, max_torque) -> float:
        """A generic test mode for the RWIP"""
        torques = namedtuple("torques", ["roll", "pitch", "yaw"])
        # TESTING: A simple control decision for testing
        # Match a proportional response to the detected angle
        amplitude = next(self.gen)
        # torques = torques(
        #     amplitude * 0.05, amplitude * 0.05, amplitude * 0.008
        # )  # , , yaw
        torques = torques(0.1,0.1,0.1)
        return torques

    def sine_sequence(self, start_x=0):
        i = 0
        while True:
            yield math.sin(start_x + i / 10)
            i += 1
