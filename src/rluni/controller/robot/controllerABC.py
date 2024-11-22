import logging
from abc import ABC, abstractmethod
from collections import namedtuple
from dataclasses import dataclass

@dataclass 
class EulerAngles:
    x: float
    y: float
    z: float

@dataclass 
class EulerRates:
    x: float
    y: float
    z: float

@dataclass
class MotorSpeedsRPM:
    pitch: float
    roll: float 
    yaw: float

# Consider how we are representing data
@dataclass
class ControlInput:
    euler_angles_x_rads: float
    euler_angles_y_rads: float
    euler_angles_z_rads: float
    euler_rates_x_rads_s: float
    euler_rates_y_rads_s: float
    euler_rates_z_rads_s: float
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
        self.num_obs = len(ControlInput.fields())
        self.num_act = 1

    @abstractmethod
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError(
                "robot_state must be an instance of RobotState from the controller module"
            )
        return 0
