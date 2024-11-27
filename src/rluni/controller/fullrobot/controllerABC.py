import logging
from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass

# Consider how we are representing data
@dataclass
class ControlInput:
    euler_angle_roll_rads: float
    euler_angle_pitch_rads: float
    euler_angle_yaw_rads: float
    euler_rate_roll_rads_s: float
    euler_rate_pitch_rads_s: float
    euler_rate_yaw_rads_s: float
    motor_speeds_roll_rads_s: float
    motor_speeds_pitch_rads_s: float
    motor_speeds_yaw_rads_s: float

class Controller(ABC):
    # This logger will not be directly used, it's here to ensure a logger is created for the base class
    _base_logger = logging.getLogger(__name__)

    def __init__(self, **kwargs):
        # Instantiate a controller logger labeled with the specific subclass name
        self.logger = logging.getLogger(f"{self.__class__.__name__}")
        # Number of observations and actions
        # self.num_obs = len(ControlInput)
        self.num_obs = 9 #TODO: Don't hardcode this, similar to above line
        self.num_act = 1

    @abstractmethod
    def get_torques(self, robot_state: ControlInput, max_torque: float) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError(
                "robot_state must be an instance of RobotState from the controller module"
            )
        return 0
