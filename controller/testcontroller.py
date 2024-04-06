from controller.controllerABC import Controller, ControlInput
       
class TestController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.MAX_TORQUE = .1
        self.logger.info(f"{self.__class__.__name__} initialized")
            
    def get_torque(self, robot_state: ControlInput, max_torque, iteration: int) -> float:
        """ A generic test mode for the RWIP 
        """
        # TESTING: A simple control decision for testing
        # Match a proportional response to the detected angle 
        setpoint = self.MAX_TORQUE * robot_state.pendulum_angle / 180  
        return setpoint