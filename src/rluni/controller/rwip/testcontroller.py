from rluni.controller.rwip.controllerABC import ControlInput, Controller
from rluni.utils.utils import call_super_first


class TestController(Controller):

    @call_super_first
    def __init__(self) -> None:
        self.MAX_TORQUE = 0.1
        self.logger.info(f"{self.__class__.__name__} initialized")

    def get_torque(self, robot_state: ControlInput, max_torque) -> float:
        """A generic test mode for the RWIP"""
        # TESTING: A simple control decision for testing
        # Match a proportional response to the detected angle
        setpoint = self.MAX_TORQUE * robot_state.pendulum_angle / 180
        return setpoint
