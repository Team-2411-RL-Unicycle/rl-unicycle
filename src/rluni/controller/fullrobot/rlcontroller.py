import numpy as np
import onnxruntime as ort

from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class RLController(Controller):

    @call_super_first
    def __init__(self, model_pth: str) -> None:
        self.model = ort.InferenceSession(model_pth)
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        list(robot_state)
        assert len(robot_state) == self.num_obs

        obs = np.zeros((1, 9))
        obs[:, 0] = robot_state.euler_angles_x_rads
        obs[:, 1] = robot_state.euler_angles_y_rads
        obs[:, 2] = robot_state.euler_angles_z_rads
        obs[:, 3] = robot_state.euler_rates_x_rads_s
        obs[:, 4] = robot_state.euler_rates_x_rads_s
        obs[:, 5] = robot_state.euler_rates_x_rads_s
        obs[:, 6] = robot_state.motor_speeds_pitch_rads_s
        obs[:, 7] = robot_state.motor_speeds_roll_rads_s
        obs[:, 8] = robot_state.motor_speeds_yaw_rads_s


        actions = self.model.run(None, {"obs": obs.astype(np.float32)})[0][0]

        assert actions.shape[0] == self.num_act

        return -np.clip(max_torque * actions, a_min=-max_torque, a_max=max_torque)[0]
