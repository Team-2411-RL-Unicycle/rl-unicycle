import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first

WHEEL_RADIUS = 0.0565 # [m]
DEGREE_TO_RAD = np.pi / 180

class PositionController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.P = 1
        self.D = 0
        self.max_bias = 3 * DEGREE_TO_RAD

    @call_super_first
    def get_torques(self, robot_state, max_torque):
        return super().get_torques(robot_state, max_torque)
    
    def get_angle_bias(self, robot_state: ControlInput, position_setpoint: float, initial_pitch: float) -> float:
        """
        Calculates the pitch angle bias by multiplying the pitch state by gains to compute
        an offset to bias the pitch angle. The biased pitch angle is to be fed into the
        low level controller causing the robot to move.
        
        Args:
            robot_state: The nominal robot state
            position_setpoint: The desired (relative) setpoint for the wheel position
            initial_pitch: The initial offset of the pitch position upon start

        Returns:
            pitch_bias (float): The required pitch angle bias to reach the setpoint.
        """

        position = (robot_state.motor_position_pitch_rads - initial_pitch) * WHEEL_RADIUS
        error = position_setpoint - position

        derror = robot_state.motor_speeds_pitch_rads_s * WHEEL_RADIUS

        pitch_bias = error * self.P + derror * self.D

        return np.clip(pitch_bias, a_min=-self.max_bias, a_max=self.max_bias)

