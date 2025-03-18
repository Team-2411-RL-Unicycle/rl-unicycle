import math
from collections import deque

from rluni.controller.fullrobot.controllerABC import ControlInput

STATE_BUFFER_SIZE = 10
PITCH_MOTOR_SPEED_LIM = 15  # rads/s
ROLL_ANGLE_LIM = 20  # degrees
PITCH_ANGLE_LIM = 25  # degrees
DEG_TO_RAD = math.pi / 180
ROLL_MOTOR_SPEED_LIM = 2500  # RPM
RPM_TO_RADS_S = 2 * math.pi / 60



class SafetyBuffer:
    """SafetyBuffer class is used to store a buffer state of the robot and evaluate the state of the robot
    for safety related decisions.

    Safe state is defined as off and resting, or on and balancing upright in both roll and pitch.
    """

    def __init__(self):
        """Initialize the SafetyBuffer object."""
        self.state_buffer = deque(maxlen=STATE_BUFFER_SIZE)
        
        # Add a single state to the buffer to avoid empty buffer checks
        self.state_buffer.append(ControlInput(0,0,0,0,0,0,0,0,0,0))

    def add_state(self, robot_state: ControlInput):
        """Add the state of the robot to the buffer."""
        self.state_buffer.append(robot_state)

    def evaluate_state(self, robot_state: ControlInput) -> tuple[bool, str]:
        """Evaluate the buffer state of the robot and return a boolean value indicating if the robot is in a safe state.

        Returns:
            safe_state (bool): True = safe, False = not safe.
        """
        self.add_state(robot_state)

        # run checks but return False if any check fails with an appropriate message
        if not self.check_pitch_speed():
            return False, "Pitch motor speed too high, shutting down."
        if not self.check_roll_spool():
            return False, "Roll motor spooled up, shutting down."
        if not self.check_tipped_over():
            return False, "Robot tipped over, shutting down."

        return True, ""

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

        return safe_state
    
    def check_tipped_over(self) -> bool:
        safe_state = False
        
        # Check if the robot is upright
        last_state = self.state_buffer[-1]        
        tipped_roll = (abs(last_state.euler_angle_roll_rads) > ROLL_ANGLE_LIM * DEG_TO_RAD)
        tipped_pitch = (abs(last_state.euler_angle_pitch_rads) > PITCH_ANGLE_LIM * DEG_TO_RAD)
        if (tipped_roll or tipped_pitch):
            safe_state = False
            return safe_state
        
        else:
            safe_state = True
            return safe_state

    def check_roll_spool(self) -> bool:
        """Check the roll wheel spool up against the angle and return a boolean value indicating if the robot is
        in a safe state. If the roll wheel has spooled up too much and has tipped over such that there is not
        enough torque to get up without stopping, the robot is not in a safe state.

        Returns:
            safe_state (bool): True = safe, False = not safe.
        """            
        safe_state = False
        
        # Check if the robot is spooled up, consecutive state above max
        spooled_up = True
        # Require one state not to be spooled up
        for state in self.state_buffer:
            if (state.motor_speeds_roll_rads_s < ROLL_MOTOR_SPEED_LIM * RPM_TO_RADS_S *.98):
                spooled_up = False
                break
        if spooled_up:
            safe_state = False
            return safe_state
        
        else:
            safe_state = True
            return safe_state
