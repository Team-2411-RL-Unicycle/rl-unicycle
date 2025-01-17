from collections import namedtuple

import numpy as np
import onnxruntime as ort

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first

torques = namedtuple("torques", ["roll", "pitch", "yaw"])


class RLController(Controller):

    @call_super_first
    def __init__(self, model_pth: str) -> None:
        self.model = ort.InferenceSession(model_pth)
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque: float):
        obs = np.zeros((1, 9))
        obs[:, 0] = robot_state.motor_speeds_roll_rads_s
        obs[:, 1] = robot_state.motor_speeds_pitch_rads_s
        obs[:, 2] = robot_state.motor_speeds_yaw_rads_s
        obs[:, 3] = robot_state.euler_angle_pitch_rads
        obs[:, 4] = robot_state.euler_angle_roll_rads
        obs[:, 5] = robot_state.euler_angle_yaw_rads
        obs[:, 6] = robot_state.euler_rate_yaw_rads_s
        obs[:, 7] = robot_state.euler_rate_pitch_rads_s
        obs[:, 8] = robot_state.euler_rate_roll_rads_s

        output = self.model.run(
            None,
            {
                "obs": obs.astype(np.float32),
            },
        )

        actions = output[0][0]

        # TODO: assert actions.shape[0] == self.num_act
        out = torques(None, None, None)
        out.roll = -np.clip(1.0 * actions[1], a_min=-max_torque, a_max=max_torque)
        out.pitch = -np.clip(1.0 * actions[0], a_min=-max_torque, a_max=max_torque)
        out.yaw = -np.clip(0.17 * actions[2], a_min=-0.17, a_max=0.17)

        return torques
