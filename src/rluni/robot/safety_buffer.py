from collections import deque
from rluni.controller.fullrobot.controllerABC import ControlInput

PITCH_MOTOR_SPEED_LIM = 15  # rads/s

class SafetyBuffer:
    """SafetyBuffer class is used to store a buffer state of the robot and evaluate the state of the robot
    for safety related decisions.

    Safe state is defined as off and resting, or on and balancing upright in both roll and pitch.
    """

    def __init__(self):
        """Initialize the SafetyBuffer object."""
        self.state_buffer = deque(maxlen=10)

    def add_state(self, robot_state: ControlInput):
        """Add the state of the robot to the buffer."""
        self.state_buffer.append(robot_state)

    def evaluate_state(self, robot_state: ControlInput) -> bool:
        """Evaluate the buffer state of the robot and return a boolean value indicating if the robot is in a safe state.
        
        Returns:
            safe_state (bool): True = safe, False = not safe.
        """
        self.add_state(robot_state)
        safe_state = self.check_pitch_speed()

        return safe_state

    def check_pitch_speed(self) -> bool:
        """Check the pitch speed of the robot and return a boolean value indicating if the robot is in a safe state.
        If the pitch wheel is moving too fast for too long, the robot is not in a safe state.

        Returns:
            safe_state (bool): True = safe, False = not safe.
        """
        safe_state = False
        for state in self.state_buffer:
            if abs(state.motor_speeds_pitch_rads_s) < PITCH_MOTOR_SPEED_LIM:
                safe_state = True
                return safe_state
        # logger.warning("Pitch motor speed too high, shutting down.")
        return safe_state
    
    def check_roll_spool(self) -> bool:
        """Check the roll wheel spool up against the angle and return a boolean value indicating if the robot is
        in a safe state. If the roll wheel has spooled up too much and has tipped over such that there is not
        enough torque to get up without stopping, the robot is not in a safe state.
        
        Returns:
            safe_state (bool): True = safe, False = not safe.        
        """
        safe_state = False
        for state in self.state_buffer:
            if state.
        return safe_state
