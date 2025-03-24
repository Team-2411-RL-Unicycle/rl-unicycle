import csv
import logging
import os
from datetime import datetime
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class CSVLogger:
    """
    Logger that writes control state and actions to a CSV file.

    Attributes:
        filename (str): The name of the CSV file to write to.
        fieldnames (List[str]): The column headers for the CSV file.
        file (file): The file object for the CSV file.
        writer (csv.DictWriter): The CSV writer object.
    """

    def __init__(self, directory: str = "log/csv_logs", filename: Optional[str] = None):
        """
        Initialize the CSV logger.

        Args:
            directory (str): Directory to save the CSV file.
            filename (str, optional): Name of the CSV file. If None, a timestamp will be used.
        """
        # Create the logs directory if it doesn't exist
        os.makedirs(directory, exist_ok=True)

        # Generate a filename with timestamp if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_control_log_{timestamp}.csv"

        self.filepath = os.path.join(directory, filename)
        self.fieldnames = [
            "timestamp",
            "iteration",
            "roll_rads",
            "pitch_rads",
            "yaw_rads",
            "roll_rate_rads",
            "pitch_rate_rads",
            "yaw_rate_rads",
            "motor_speed_roll_rads",
            "motor_speed_pitch_rads",
            "motor_speed_yaw_rads",
            "motor_position_pitch_rads",
            "torque_roll",
            "torque_pitch",
            "torque_yaw",
            "is_calibrating",
        ]

        self.file = None
        self.writer = None
        logger.info(f"CSV logger initialized. Will write to {self.filepath}")

    def open(self):
        """Open the CSV file and write the header."""
        try:
            self.file = open(self.filepath, "w", newline="")
            self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
            self.writer.writeheader()
            logger.info(f"CSV file opened: {self.filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to open CSV file: {e}")
            return False

    def log(self, data: Dict):
        """
        Log a row of data to the CSV file.

        Args:
            data (Dict): Dictionary containing the data to log.
        """
        if self.writer is None:
            logger.error("CSV logger not initialized. Call open() first.")
            return False

        try:
            self.writer.writerow(data)
            return True
        except Exception as e:
            logger.error(f"Failed to write to CSV file: {e}")
            return False

    def close(self):
        """Close the CSV file."""
        if self.file:
            self.file.close()
            logger.info(f"CSV file closed: {self.filepath}")
            self.file = None
            self.writer = None
