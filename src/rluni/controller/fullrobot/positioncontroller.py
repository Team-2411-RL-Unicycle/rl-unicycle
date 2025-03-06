from collections import namedtuple
from importlib.resources import files

import numpy as np

from rluni.controller.fullrobot import LQRController
from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils import get_validated_config_value as gvcv
from rluni.utils.utils import call_super_first, load_config_file

DEGREE_TO_RAD = np.pi / 180


class PositionController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.logger.info(f"{self.__class__.__name__} initialized.")
        self.lqr_controller = LQRController()
        self.P = 0.2
        self.D = -0.2
        self.max_bias = 5 * DEGREE_TO_RAD
        self.initial_pitch = None
        config = self._load_config("unicycle.yaml")
        self.PITCH_RADIUS = gvcv(
            config, "RobotSystem.pitch_wheel_radius", float, required=True
        )

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque):
        """
        Get the torques to apply to the robot to reach the desired position.

        First, the pitch angle is updated based on the pitch wheel position and speed.
        Then, the LQR controller is used to compute the torques to apply to the robot.

        Args:
            robot_state: The current state of the robot.
            max_torque: The maximum torque that can be applied to the robot.

        Returns:
            torques: The torques to apply to the robot.
        """
        # TODO: Handle the MQTT message to set the pitch setpoint (similar to xbox)

        if not self.initial_pitch:
            self.initial_pitch = robot_state.motor_position_pitch_rads

        pitch_bias = self._update_pitch(
            robot_state.motor_position_pitch_rads,
            robot_state.motor_speeds_pitch_rads_s,
            0,
            self.initial_pitch,
        )
        print("get torques: bias: ", pitch_bias)

        robot_state.euler_angle_pitch_rads += pitch_bias

        roll, pitch, yaw = self.lqr_controller.get_torques(robot_state, max_torque)

        torques = namedtuple("torques", ["roll", "pitch", "yaw"])
        return torques(roll, pitch, yaw)

    def _load_config(self, config_file):
        """
        Private method to load and parse the robot configuration file.
        """
        # Load the robot configuration file (use the default if none provided)
        if config_file is None:
            config_file = "unicycle.yaml"

        config_file_path = files("rluni.configs.robot").joinpath(config_file)
        config_file = str(config_file_path)

        config = load_config_file(config_file)
        return config

    def _update_pitch(
        self,
        motor_position_pitch_rads: float,
        motor_speeds_pitch_rads_s: float,
        position_setpoint: float,
        initial_pitch: float,
    ) -> float:
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

        position = (motor_position_pitch_rads - initial_pitch) * self.PITCH_RADIUS
        error = position_setpoint - position

        print("_update_pitch: position", position)

        # target velocity always zero
        derror = motor_speeds_pitch_rads_s * self.PITCH_RADIUS

        prop_error = error * self.P
        deriv_error = derror * self.D

        print("P * error: ", prop_error)
        print("D * derror: ", deriv_error)

        deriv_error = np.clip(deriv_error, a_min=-0.02, a_max=0.02)

        print("clipped D * derror: ", deriv_error)

        pitch_bias = error * self.P + derror * self.D
        self.logger.info(f"bias = {pitch_bias}")

        return np.clip(pitch_bias, a_min=-self.max_bias, a_max=self.max_bias)
