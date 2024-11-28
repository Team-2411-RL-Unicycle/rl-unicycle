import numpy as np
import onnxruntime as ort

from rluni.controller.rwip.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class RLController(Controller):

    @call_super_first
    def __init__(self, model_pth: str) -> None:
        self.model = ort.InferenceSession(model_pth)
        self.logger.info(f"{self.__class__.__name__} initialized")

    @call_super_first
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        #        list(robot_state)
        #        assert len(robot_state) == self.num_obs

        obs = np.zeros((1, 3))
        obs[:, 0] = robot_state.wheel_vel * -1.0  # redefine to be positive CCW
        obs[:, 1] = robot_state.pendulum_angle
        obs[:, 2] = robot_state.pendulum_vel

        actions = self.model.run(None, {"obs": obs.astype(np.float32)})[0][0]

        assert actions.shape[0] == self.num_act

        return -np.clip(max_torque * actions, a_min=-max_torque, a_max=max_torque)[0]
