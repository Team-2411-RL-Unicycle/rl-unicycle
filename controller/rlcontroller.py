from controller.controllerABC import Controller, ControlInput
#TODO add torch to the venve so that this import works (math as torch placeholder)
import onnxruntime as ort
import numpy as np


class RLController(Controller):
    def __init__(self, model_pth: str) -> None:
        super().__init__()
        self.model = ort.InferenceSession(model_pth)
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        super().get_torque(robot_state)
        list(robot_state)
        assert len(robot_state) == self.num_obs

        obs = np.zeros((1, 3))
        obs[:, 0] = robot_state[2]
        obs[:, 1] = robot_state[0]
        obs[:, 2] = robot_state[1]

        actions = self.model.run(
            None,
            {"obs": obs.astype(np.float32)}
        )[0][0]
        
        assert actions.shape[0] == self.num_act

        return np.clip(max_torque * actions, a_min=-max_torque, a_max=max_torque)[0]