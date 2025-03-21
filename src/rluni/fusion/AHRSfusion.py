import logging

import imufusion
import numpy as np
import yaml
from transforms3d import euler, quaternions

from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

# Create a logger for your module
logger = logging.getLogger(__name__)

"""
AHRS sensor fusion algorithm from  https://github.com/xioTechnologies/Fusion
"""


class AHRSfusion:

    def __init__(self, sample_rate, config_file):
        # Initialize the IMU fusion parameters
        self.sample_rate = sample_rate
        self._gyro_range = None
        self.transformation_matrix = np.identity(3)

        self.euler_angles = (0, 0, 0)  # x, y, z
        self.euler_rates = (0, 0, 0)

        # Load the configuration file
        self.config = load_config_file(config_file)
        self._load_config()

        # Instantiate algorithms
        self.offset = imufusion.Offset(self.sample_rate)

        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,  # convention
            1.0,  # 0.5 gain (on the accel error correction)
            self._gyro_range,  # gyroscope range
            10,  # acceleration rejection
            0,  # magnetic rejection
            5 * self.sample_rate,
        )  # recovery trigger period = 5 seconds

    def _load_config(self):
        """
        Load and validate the ICM20948 configuration and calibration settings.
        """
        # Access the ICM20948 configuration
        self._gyro_range = gvcv(
            self.config, "ICM20948_Configuration.gyro_range", int, required=True
        )

        # Parse the rotation matrix from the 'Calibration' section
        rotation_list = gvcv(self.config, "Calibration.rotation", list, required=True)
        rot_mat = np.array(rotation_list).reshape(3, 3)
        x_angle = gvcv(self.config, "Calibration.x_adj", float, required=True)
        y_angle = gvcv(self.config, "Calibration.y_adj", float, required=True)

        # Compute the transformation matrix
        self.transformation_matrix = self._calculate_transformation_matrix(
            rot_mat, x_angle, y_angle
        )

        logger.debug(f"Loaded rotation matrix:\n{self.transformation_matrix}")

    def _calculate_transformation_matrix(self, rot_mat, pitch_angle, roll_angle):
        pitch_angle = np.radians(pitch_angle)
        roll_angle = np.radians(roll_angle)

        # Rotation about the x-axis
        x_rot = np.array(
            [
                [1, 0, 0],
                [0, np.cos(pitch_angle), -np.sin(pitch_angle)],
                [0, np.sin(pitch_angle), np.cos(pitch_angle)],
            ]
        )

        # Rotation about the y-axis
        y_rot = np.array(
            [
                [np.cos(roll_angle), 0, np.sin(roll_angle)],
                [0, 1, 0],
                [-np.sin(roll_angle), 0, np.cos(roll_angle)],
            ]
        )

        M = rot_mat @ x_rot @ y_rot
        return M

    def rotate_frame_imu_to_robot(self, x, y, z):
        """
        Input: x, y, z in IMU frame
        Output: x', y', z' in robot frame"""
        return self.transformation_matrix @ np.array([x, y, z])

    def update_state_from_quaternion(self, gyro_data):
        """Compute and return ZYX Euler angles and rotation matrix A q_dot = gyro"""
        angles = euler.quat2euler(self.ahrs.quaternion.wxyz, axes="rzxy")
        self.euler_angles = np.rad2deg(angles)

        z, x, y = angles  # In radians
        mat = np.array(
            [
                [np.cos(y), 0, np.sin(y)],
                [1.0 * np.sin(y) * np.tan(x), 1, -1.0 * np.cos(y) * np.tan(x)],
                [-np.sin(y) / np.cos(x), 0, np.cos(y) / np.cos(x)],
            ]
        )
        self.euler_rates = mat @ (
            gyro_data * np.pi / 180
        )  # convert from deg/s to rad/s before multiplying

        # NOTE: The angles are in degrees and rates are in rads/s

    def update(self, gyro_data, accel_data, mag_data=None, delta_time=0.001):

        # Convert to numpy arrays and rotate to robot frame
        accel_data = np.array(self.rotate_frame_imu_to_robot(*accel_data))
        gyro_data = np.array(self.rotate_frame_imu_to_robot(*gyro_data))

        if mag_data is None or mag_data[0] is None:
            mag_data = np.array([0, 0, 0])
        else:
            mag_data = np.array(mag_data)

        mag_data = np.array(self.rotate_frame_imu_to_robot(*mag_data))

        # Update gyro data with offset (this is dynamic bias correction for gyro)
        corrected_gyro = self.offset.update(gyro_data)

        # Update the AHRS algorithm with the new data
        self.ahrs.update(corrected_gyro, accel_data, mag_data, delta_time)

        # Retrieve internal states and flags for logging or further processing
        internal_states = np.array(
            [
                self.ahrs.internal_states.acceleration_error,
                self.ahrs.internal_states.accelerometer_ignored,
                self.ahrs.internal_states.acceleration_recovery_trigger,
                self.ahrs.internal_states.magnetic_error,
                self.ahrs.internal_states.magnetometer_ignored,
                self.ahrs.internal_states.magnetic_recovery_trigger,
            ]
        )

        flags = np.array(
            [
                self.ahrs.flags.initialising,
                self.ahrs.flags.angular_rate_recovery,
                self.ahrs.flags.acceleration_recovery,
                self.ahrs.flags.magnetic_recovery,
            ]
        )

        self.update_state_from_quaternion(gyro_data)

        return self.ahrs.quaternion, internal_states, flags
