from collections import deque
from rluni.controller.fullrobot.controllerABC import ControlInput


class SafetyBuffer:
    """SafetyBuffer class is used to store the state of the robot and evaluate the state of the robot
    for safety related decisions.
    """

    def __init__(self):
        """Initialize the SafetyBuffer object."""
        self.state_buffer = deque(maxlen=10)

    def add_state(self, robot_state: ControlInput):
        """Add the state of the robot to the buffer."""
        self.state_buffer.append(robot_state)

    def evaluate_state(self, robot_state: ControlInput) -> bool:
        """Evaluate the state of the robot and return a boolean value indicating if the robot is in a safe state.
        
        Returns:
            safe_state (bool): True = safe, False = not safe.
        """
        self.add_state(robot_state)

        # Check if the robot is in a safe state
        safe_state = self.check_pitch_speed()

        return safe_state

    def check_pitch_speed(self) -> bool:
        """Check the pitch speed of the robot and return a boolean value indicating if the robot is in a safe state."""
        safe_state = False
        for state in self.state_buffer:
            if abs(state.motor_speeds_pitch_rads_s) < 15:
                safe_state = True
                break
        return safe_state
