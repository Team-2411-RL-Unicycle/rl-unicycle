from controller.controllerABC import Controller, ControlInput
from simple_pid import PID
       
class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.MAX_TORQUE = 0.1
        self._Kp = 0.01
        self._Ki = 0.0 
        self._Kd = 0.0
        self._setpoint = 0.0
        self._pid = PID(Kp, Ki, Kd, setpoint=setpoint)
        self.logger.info(f"{self.__class__.__name__} initialized")
            
    def get_torque(self, robot_state: ControlInput) -> float:
        """
        Calculates a torque using the PID error of pendulum angle. If the calculated torque 
        is greater than the specified maximum, a warning message will be logged and the torque
        will be clamped. 

        Returns: 
            torque: Desired torque for the PID controller. 
        """
        super().get_torque(robot_state)
        # Calculate torque
        torque = self._pid(robot_state.pendulum_angle)
        # Clamp torque if outside bounds
        if abs(torque) > self.MAX_TORQUE: 
            warning_msg = f'WARNING: PID controller set torque outside bounds. Attempted to set to {torque:.3f} 
            N*m at {robot_state.pendulum_angle} degrees. Bounds are +/- {self.MAX_TORQUE}' 
            logger.warning(warning_msg)
            print(warning_msg) # TODO: Remove this for performance? 
            torque = self.MAX_TORQUE * (1 if torque > 0 else -1)
        return torque

    def update_parameter(param: str, value: float): 
        """
        Update PID Controller's control parameter to a new value.

        Args: 
            param: must be 'P', 'I', or 'D'
            value: value of parameter
        """
        if param == 'P': 
            self.Kp = value
        elif param == 'I': 
            self.Ki = value
        elif param == 'D': 
            self.Kd = value
        # Raise error if not P, I, or D
        else: 
            Raise ValueError(f"Invalid parameter: {param}")
        return

        