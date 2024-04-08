from controller.controllerABC import Controller, ControlInput
import numpy as np
from simple_pid import PID
from typing import Tuple

class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self._Kp = 0.07
        self._Ki = 0.00
        self._Kd = 0.0038
        self._Kp_wheel_vel = .2 #0.1 - 0.2 is a potential value range to check
        self._Ki_wheel_vel = .03
        self._Kd_wheel_vel = 0.0065
        self._max_rps = 35  # revs/s
        self._max_del_s = 2.0  # degrees
        self._pid = PID(self._Kp, self._Ki, self._Kd, setpoint=0)
        self.anti_windup_timer = 0
        self.logger.info(f"{self.__class__.__name__} initialized")
        self._pid_wheel = PID(self._Kp_wheel_vel, 0, 0, setpoint=0)
            
    def get_torque(self, robot_state: ControlInput, max_torque: float, iteration: int) -> Tuple[float, bool]:
        """
        Calculates a torque using the PID error of pendulum angle. If the calculated torque 
        is greater than the specified maximum, a warning message will be logged and the torque
        will be clamped. 

        Returns: 
            torque: Desired torque for the PID controller.
        """
        super().get_torque(robot_state, max_torque, iteration)
        # Check for saturation to engage anti windup
        anti_windup = self.anti_windup(robot_state)
        # Update control setpoint based on wheel velocity
        self.update_control_setpoint(robot_state.wheel_vel)
        # Calculate torque
        torque = self._pid(robot_state.pendulum_angle-.4)
        # Clamp torque if outside bounds
        if abs(torque) > max_torque: 
            torque = max_torque * (1 if torque > 0 else -1)

        return torque, anti_windup
    
    def update_control_setpoint(self, wheel_vel: float) -> float: 
        """
        Calculate the setpoint for the PID controller based on the wheel velocity. 
        
        Theory: create a saturating P controller in response to deviation from 0 velocity on wheel
        The controller saturates at max_del_s degrees of pendulum angle.
        """           
        setpoint = -self._pid_wheel(wheel_vel)
        self._pid.setpoint = np.clip(setpoint, -self._max_del_s, self._max_del_s)      
      
    def update_parameter(self, param: str, value: float): 
        """
        Update PID Controller's control parameter to a new value.

        Args: 
            param: must be 'P', 'I', or 'D'
            value: value of parameter
        """
        if param == 'P': 
            self._Kp_wheel_vel = value
        elif param == 'I': 
            self._Ki_wheel_vel = value
        elif param == 'D': 
            self._Kd_wheel_vel = value
        # Raise error if not P, I, or D
        else: 
            raise ValueError(f"Invalid parameter: {param}")

        self._pid_wheel.reset()
        self._pid_wheel.tunings = (self._Kp_wheel_vel, self._Ki_wheel_vel, self._Kd_wheel_vel)
        return
    
    def anti_windup(self, robot_state: ControlInput) -> bool:
        """
        Anti-windup scheme to prevent integration wind up when an actuator is saturated.

        Theory: if we are saturated, faster rotation won't yield torque. Stopping and setting a flag
        for 1 second to then "build up" our torque reserve will allow balancing again.

        Parameters:
            robot_state: used to access wheel_vel in [rev/s] and pendulum_angle in [deg]

        Returns:
            True while waiting for anti_windup timer to count down
        """
        if self.anti_windup_timer > 0:
            self.anti_windup_timer -= 1
            return True

        # print(f"abs(robot_state.wheel_vel) = {abs(robot_state.wheel_vel)}")
        # print(f"abs(robot_state.pendulum_angle) = {abs(robot_state.pendulum_angle)}")
        if ((robot_state.wheel_vel > self._max_rps and
                robot_state.pendulum_angle > 25) or
            (robot_state.wheel_vel < -self._max_rps and
                robot_state.pendulum_angle < -25)
            ):
            self.anti_windup_timer = 1200
            return True
        return False
