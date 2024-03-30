from controller.controllerABC import Controller, ControlInput
from simple_pid import PID
       
class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.MAX_TORQUE = 0.1
        self.Kp = 0.01
        self.Ki = 0.0 
        self.Kd = 0.0
        self.setpoint = 0.0
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
        torque = self._pid(robot_state.pendulum_angle)
        if abs(torque) > self.MAX_TORQUE: 
            warning_msg = f'PID controller set torque outside bounds. Attempted to set to {torque:.3f} 
            N*m at {robot_state.pendulum_angle} degrees. Bounds are +/- {self.MAX_TORQUE}' 
            logger.warning(warning_msg)
            torque = MAX_TORQUE * (1 if torque > 0 else -1)
        return torque

        