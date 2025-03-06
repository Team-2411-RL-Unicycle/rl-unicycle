import logging

from rluni.controller.fullrobot import Controller

logger = logging.getLogger(__name__)


class YawController(Controller):
    """
    The YawController listens for yaw commands from the MQTT stream and applies them to the yaw motor.
    """

    def __init__(self, max_torque_yaw):
        """
        Initialize the yaw controller.

        Args:
            max_torque_yaw (float): The maximum allowable torque for the yaw motor.
        """
        super().__init__()
        self.max_torque_yaw = max_torque_yaw
        self.current_torque = 0.0

    def update_yaw_command(self, yaw_torque: float):
        """
        Update the yaw torque command based on the received input.

        Args:
            yaw_torque (float): The desired yaw torque command from MQTT.
        """
        yaw_torque = yaw_torque * self.max_torque_yaw

        if abs(yaw_torque) > self.max_torque_yaw:
            logger.warning(
                f"Yaw torque command {yaw_torque} exceeds max torque {self.max_torque_yaw}. Clipping."
            )
            yaw_torque = max(-self.max_torque_yaw, min(yaw_torque, self.max_torque_yaw))

        self.current_torque = yaw_torque
        logger.info(f"Yaw torque updated to: {self.current_torque}")

    def get_torques(self, *args, **kwargs):
        """
        Returns the current yaw torque.

        Returns:
            tuple: A tuple with (0.0, 0.0, current_torque) as we only control yaw here.
        """
        return self.current_torque
