import csv
import logging
import os

logger = logging.getLogger(__name__)

class CSVLogger:
    """
    Logs control loop data to a CSV file. Writes iteration number, control input values, 
    and torques on each loop cycle while keeping the file open for efficiency.

    The file is automatically closed on shutdown.

    Attributes:
        filename (str): The path of the CSV file.
        enabled (bool): Whether logging is active.
    """

    def __init__(self, filename="robot_log.csv", enabled=False):
        """
        Initialize the CSV logger.

        Args:
            filename (str): The path to the CSV file.
            enabled (bool): If True, logging is enabled; otherwise, no data is written.
        """
        self.filename = filename
        self.enabled = enabled
        self.file = None
        self.writer = None

        if self.enabled:
            self._open_file()

    def _open_file(self):
        """Open the CSV file and prepare for writing."""
        try:
            # Ensure the directory exists
            os.makedirs(os.path.dirname(self.filename), exist_ok=True)

            # Open file in append mode and create a writer
            self.file = open(self.filename, mode="a", newline="")
            self.writer = csv.writer(self.file)

            # If file is empty, write the header
            if self.file.tell() == 0:
                self.writer.writerow(
                    [
                        "Iteration",
                        "Euler_Roll", "Euler_Pitch", "Euler_Yaw",
                        "EulerRate_Roll", "EulerRate_Pitch", "EulerRate_Yaw",
                        "MotorSpeed_Pitch", "MotorSpeed_Roll", "MotorSpeed_Yaw",
                        "MotorTorque_Roll", "MotorTorque_Pitch", "MotorTorque_Yaw"
                    ]
                )

            logger.info(f"CSV logging enabled, writing to {self.filename}")
        except Exception as e:
            logger.error(f"Failed to open CSV file: {e}")
            self.enabled = False

    def log(self, iteration, control_input, torques):
        """
        Write a new row to the CSV file with control input and torques.

        Args:
            iteration (int): The control loop iteration number.
            control_input (ControlInput): The current control input state.
            torques (tuple): A tuple of (roll_torque, pitch_torque, yaw_torque).
        """
        if not self.enabled or self.writer is None:
            return

        try:
            self.writer.writerow(
                [
                    iteration,
                    control_input.euler_angle_roll_rads,
                    control_input.euler_angle_pitch_rads,
                    control_input.euler_angle_yaw_rads,
                    control_input.euler_rate_roll_rads_s,
                    control_input.euler_rate_pitch_rads_s,
                    control_input.euler_rate_yaw_rads_s,
                    control_input.motor_speeds_pitch_rads_s,
                    control_input.motor_speeds_roll_rads_s,
                    control_input.motor_speeds_yaw_rads_s,
                    torques.roll,
                    torques.pitch,
                    torques.yaw,
                ]
            )
        except Exception as e:
            logger.error(f"Error writing to CSV: {e}")

    def close(self):
        """Close the CSV file properly."""
        if self.file:
            try:
                self.file.close()
                logger.info("CSV file closed.")
            except Exception as e:
                logger.error(f"Error closing CSV file: {e}")
            finally:
                self.file = None
                self.writer = None
