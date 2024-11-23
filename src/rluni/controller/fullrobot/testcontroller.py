from rluni.controller.fullrobot.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class TestController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.MAX_TORQUE = 0.1
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torques(self, robot_state: ControlInput, max_torque) -> float:
        """A generic test mode for the RWIP"""
        # TESTING: A simple control decision for testing
        # Match a proportional response to the detected angle
        torques = [0.05,0.05,0.005] # , , yaw
        return torques
        
