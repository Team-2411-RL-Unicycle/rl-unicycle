from controller.controllerABC import Controller, ControlInput
import numpy as np
import onnxruntime as ort
from typing import Tuple

class RLController(Controller):
    def __init__(self, model_pth: str) -> None:
        super().__init__()
        self.model = ort.InferenceSession(model_pth)
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torque(self, robot_state: ControlInput, max_torque: float, iteration: int) -> Tuple[float, bool]:
        super().get_torque(robot_state, max_torque, iteration)
        list(robot_state)
        assert len(robot_state) == self.num_obs

        obs = np.zeros((1, 3))
        obs[:, 0] = robot_state.wheel_vel * 2 * np.pi
        obs[:, 1] = robot_state.pendulum_angle * np.pi / 180
        obs[:, 2] = robot_state.pendulum_vel * np.pi / 180

        actions = self.model.run(
            None,
            {"obs": obs.astype(np.float32)}
        )[0][0]
        
        assert actions.shape[0] == self.num_act

        return np.clip(max_torque * actions, a_min=-max_torque, a_max=max_torque)[0], False

    def anti_windup(self, robot_state: ControlInput) -> bool:
        return super().anti_windup(robot_state)