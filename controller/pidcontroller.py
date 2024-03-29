from controller.controllerABC import Controller, ControlInput
       
class PIDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.logger.info(f"{self.__class__.__name__} initialized")
            
    def get_torque(self, robot_state: ControlInput) -> float:
        return super().get_torque(robot_state)