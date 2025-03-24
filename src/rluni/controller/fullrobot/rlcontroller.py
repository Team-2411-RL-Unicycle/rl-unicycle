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
        self.rnn = True
        self.prev_action = np.zeros((2,)).astype(np.float32)
        self.ema = 0.0
        if self.rnn:
            self.out_state = np.zeros((1, 1, 32)).astype(np.float32)
            self.hidden_state = np.zeros((1, 1, 32)).astype(np.float32)

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque: float):
        obs = np.zeros((1, 8))
        obs[:, 0] = robot_state.motor_speeds_roll_rads_s/2.5
        obs[:, 1] = robot_state.motor_speeds_pitch_rads_s/2.5
        obs[:, 2] = robot_state.motor_speeds_yaw_rads_s*0
        obs[:, 3] = robot_state.euler_angle_roll_rads - 0.2*np.pi/180 # -0.2 is current best
        obs[:, 4] = robot_state.euler_angle_pitch_rads + 2.9*np.pi/180 # + 3.0 current best, 3.3 similar
        # obs[:, 5] = robot_state.euler_angle_yaw_rads
        obs[:, 6] = robot_state.euler_rate_roll_rads_s
        obs[:, 7] = robot_state.euler_rate_pitch_rads_s
        obs[:, 5] = robot_state.euler_rate_yaw_rads_s

        output = self.model.run(
            None,
            {
                "obs": obs.astype(np.float32),
                "out_state.1": self.out_state,
                "hidden_state.1": self.hidden_state,
            },
        )

        actions = output[0][0]
        sigmas = np.exp(output[1][0])

        # roll_action_scaled = np.random.normal(actions[0], sigmas[0])
        # pitch_action_scaled = np.random.normal(actions[1], sigmas[1])

        roll_action_scaled = actions[0]
        pitch_action_scaled = actions[1]

        self.out_state = output[3]
        self.hidden_state = output[4]
        max_torque = 1.4

        filtered_action = self.compute_ema(np.array([roll_action_scaled, pitch_action_scaled]))
        self.prev_action = filtered_action

        roll = -np.clip(2.0 * filtered_action[0], a_min=-max_torque, a_max=max_torque)
        pitch = -np.clip(2.0 * filtered_action[1], a_min=-max_torque, a_max=max_torque)/6
        # yaw = -np.clip(0.17 * actions[2], a_min=-0.17, a_max=0.17)
        yaw = 0.0
        out = torques(roll, pitch, yaw)

        return out

    def compute_ema(self, next_action):
        return self.ema * self.prev_action + (1 - self.ema) * next_action