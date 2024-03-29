from controller.controllerABC import Controller, ControlInput
#TODO add torch to the venve so that this import works (math as torch placeholder)
import math as torch

class RLController(Controller):
    def __init__(self, model_pth: str) -> None:
        super().__init__()
        self.model = torch.load(model_pth)
        self.model.eval()
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torque(self, robot_state: ControlInput) -> float:
        super().get_torque(robot_state)
        list(robot_state)
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