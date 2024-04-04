from controller.controllerABC import Controller, ControlInput
from simple_pid import PID
import numpy as np
       
class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self._Kp = -0.08
        self._Ki = 0.0005
        self._Kd = 0.0000
        self._setpoint = 0.0
        self.max_rps = 35  # revs/s
        self.max_del_s = 3  # degrees
        self._pid = PID(self._Kp, self._Ki, self._Kd, setpoint=self._setpoint)
        self.logger.info(f"{self.__class__.__name__} initialized")
            
    def get_torque(self, robot_state: ControlInput, max_torque: float) -> float:
        """
        Calculates a torque using the PID error of pendulum angle. If the calculated torque 
        is greater than the specified maximum, a warning message will be logged and the torque
        will be clamped. 

        Returns: 
            torque: Desired torque for the PID controller. 
        """
        super().get_torque(robot_state)
        # update setpoint
        new_setpoint = -self.max_del_s*np.tanh(robot_state.pendulum_vel/10)
        # new_setpoint = -self.max_del_s/self.max_rps * robot_state.pendulum_vel
        # new_setpoint = np.clip(new_setpoint, -self.max_del_s, self.max_del_s) 
        
        self._pid.setpoint = new_setpoint

        # Calculate torque
        torque = self._pid(robot_state.pendulum_angle)
        # Clamp torque if outside bounds
        if abs(torque) > max_torque: 
            # TODO: Throw error if outside bounds
            warning_msg = f'WARNING: PID controller set torque outside bounds. Attempted to set to {torque:.3f} N*m at {robot_state.pendulum_angle} degrees. Bounds are +/- {max_torque}' 
            self.logger.warning(warning_msg)
            print(warning_msg) # TODO: Remove this for performance? 
            torque = max_torque * (1 if torque > 0 else -1)
            # raise ValueError("Torque outside bounds")
        return torque

    def update_parameter(self, param: str, value: float): 
        """
        Update PID Controller's control parameter to a new value.

        Args: 
            param: must be 'P', 'I', or 'D'
            value: value of parameter
        """
        if param == 'P': 
            self._Kp = value
        elif param == 'I': 
            self._Ki = value
        elif param == 'D': 
            self._Kd = value
        # Raise error if not P, I, or D
        else: 
            raise ValueError(f"Invalid parameter: {param}")

        self._pid = PID(self._Kp, self._Ki, self._Kd, setpoint=self._setpoint)
        return

        
