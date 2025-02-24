import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first

WHEEL_RADIUS = 0.0565 # [m]

class PositionController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.P = 1
        self.D = 0

    @call_super_first
    def get_torques(self, robot_state, max_torque):
        return super().get_torques(robot_state, max_torque)
    
    def get_angle_bias(self, robot_state: ControlInput, position_setpoint: float) -> float:
        """
        Calculates the pitch angle bias by multiplying the pitch state by gains to compute
        an offset to bias the pitch angle. The biased pitch angle is to be fed into the
        low level controller causing the robot to move.

        Returns:
            pitch_bias (float): The required pitch angle bias to reach the setpoint.
        """

        position = robot_state.motor_position_pitch_rads * WHEEL_RADIUS
        error = position_setpoint - position

        derror = robot_state.motor_speeds_pitch_rads_s * WHEEL_RADIUS

        pitch_bias = error * self.P + derror * self.D

