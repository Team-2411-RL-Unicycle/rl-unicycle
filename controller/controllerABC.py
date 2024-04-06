import logging
from abc import ABC, abstractmethod
from collections import namedtuple
ControlInput = namedtuple('ControlInput', ['pendulum_angle', 'pendulum_vel', 'wheel_vel'])

class Controller(ABC):
    # This logger will not be directly used, it's here to ensure a logger is created for the base class
    _base_logger = logging.getLogger(__name__)

    def __init__(self):
        # Instantiate a controller logger labeled with the specific subclass name
        self.logger = logging.getLogger(f'{self.__class__.__name__}')
        # Number of observations and actions
        self.num_obs = 3
        self.num_act = 1
        self._max_rps = 35  # revs/s
        self.anti_windup_timer = 0

    @abstractmethod
    def get_torque(self, robot_state: ControlInput, max_torque: float, iteration: int) -> float:
        if not isinstance(robot_state, ControlInput):
            raise TypeError("robot_state must be an instance of RobotState from the controller module")
        return 0

    @abstractmethod
    def anti_windup(self, anti_windup_timer: int, robot_state: ControlInput) -> bool:
        """
        Anti-windup scheme to prevent integration wind up when an actuator is saturated.

        Theory: if we are saturated, faster rotation won't yield torque. Stopping and setting a flag
        for 1 second to then "build up" our torque reserve will allow balancing again.

        Parameters:
            anti_windup_timer: the count of how many cycles left in the anti windup
            robot_state: used to access wheel_vel in [rev/s] and pendulum_angle in [deg]

        Returns:
            True while waiting for anti_windup timer to count down
        """
        if anti_windup_timer > 0:
            self.anti_windup_timer -= 1
            return True

        if (abs(robot_state.wheel_vel) > 0.9*self._max_rps and
                robot_state.pendulum_angle > 20
            ):
            self.anti_windup_timer = 100
            return True

        return False
