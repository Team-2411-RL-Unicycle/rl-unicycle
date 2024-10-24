import logging
from abc import ABC, abstractmethod
from collections import namedtuple

ControlInput = namedtuple(
    "ControlInput", ["pendulum_angle", "pendulum_vel", "wheel_vel", "roll_torque"]
)


class Controller(ABC):
    # This logger will not be directly used, it's here to ensure a logger is created for the base class
    _base_logger = logging.getLogger(__name__)

    def __init__(self, **kwargs):
        # Instantiate a controller logger labeled with the specific subclass name
        self.logger = logging.getLogger(f"{self.__class__.__name__}")
        # Number of observations and actions
        self.num_obs = len(ControlInput._fields)
        self.num_act = 1

    @abstractmethod
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError(
                "robot_state must be an instance of RobotState from the controller module"
            )
        return 0
