from collections import namedtuple
import numpy as np

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.controller.fullrobot import LQRController
from rluni.utils.utils import call_super_first, load_config_file
from rluni.utils import get_validated_config_value as gvcv

DEGREE_TO_RAD = np.pi / 180


class PositionController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.logger.info(f"{self.__name__.__class__} initialized.")
        self.lqr_controller = LQRController()
        self.P = 1
        self.D = 0
        self.max_bias = 3 * DEGREE_TO_RAD
        self.initial_pitch = None
        config = load_config_file("unicycle.yaml")
        self.PITCH_RADIUS = gvcv(
            config, "RobotSystem.pitch_wheel_radius", float, required=True)

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque):
        if not self.initial_pitch:
            self.initial_pitch = robot_state.motor_position_pitch_rads

        robot_state.euler_angle_pitch_rads -= self._update_pitch(
            robot_state.motor_position_pitch_rads,
            robot_state.motor_speeds_pitch_rads_s,
            0,
            self.initial_pitch)

        roll, pitch, yaw = self.lqr_controller.get_torques(robot_state, max_torque)

        torques = namedtuple("torques", ["roll", "pitch", "yaw"])
        return torques(roll, pitch, yaw)

    def _update_pitch(self,
                      motor_position_pitch_rads: float,
                      motor_speeds_pitch_rads_s: float,
                      position_setpoint: float,
                      initial_pitch: float) -> float:
        """
        Calculates the pitch euler angle bias by multiplying the pitch state by gains to compute
        an offset to bias the pitch angle. The biased pitch angle is to be fed into the
        low level controller causing the robot to move.

        Args:
            robot_state: The nominal robot state
            position_setpoint: The desired (relative) setpoint for the wheel position
            initial_pitch: The initial offset of the pitch position upon start

        Returns:
            pitch_bias (float): The required pitch angle bias to reach the setpoint.
        """

        position = (motor_position_pitch_rads -
                    initial_pitch) * self.PITCH_RADIUS
        error = position_setpoint - position

        # target velocity always zero
        derror = motor_speeds_pitch_rads_s * self.PITCH_RADIUS

        pitch_bias = error * self.P + derror * self.D

        return np.clip(pitch_bias, a_min=-self.max_bias, a_max=self.max_bias)
