from abc import ABC, abstractmethod

import torch


class Controller(ABC):
    @abstractmethod
    def get_torque(self, robot_state: list):
        pass

class RLController(Controller):
    def __init__(self, model_pth: str) -> None:
        super().__init__()
        self.model = torch.load(model_pth)
        self.model.eval()

        self.num_obs = 3
        self.num_act = 1

    def get_torque(self, robot_state: list) -> float:
        assert len(robot_state) == self.num_obs

        #TODO: Check which device we should use
        state = torch.zeros(size=(1, self.num_obs))

        # Loop here for future if preprocessing necessary
        for i, obs in enumerate(robot_state):
            #TODO: Do tensors for trained policy need a batch size dim
            state[:, i] = obs

        action = self.model(state)
        assert action.shape[1] == self.num_act

        return action[0]