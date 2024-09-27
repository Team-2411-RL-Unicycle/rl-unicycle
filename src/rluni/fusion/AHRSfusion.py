import logging

import imufusion
import numpy as np

# Create a logger for your module
logger = logging.getLogger(__name__)

"""
AHRS sensor fusion algorithm from  https://github.com/xioTechnologies/Fusion
"""


class AHRSfusion:

    def __init__(self, gyro_range, sample_rate):
        # Initialize the IMU fusion parameters
        self.sample_rate = sample_rate
        self.gyro_range = gyro_range

        # Instantiate algorithms
        self.offset = imufusion.Offset(self.sample_rate)

        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,  # convention
            0.5,  # 0.5 gain (on the accel error correction)
            self.gyro_range,  # gyroscope range
            1,  # acceleration rejection
            0,  # magnetic rejection
            5 * self.sample_rate,
        )  # recovery trigger period = 5 seconds

    def update(self, gyro_data, accel_data, mag_data=None, delta_time=0.001):
        # Change alignment to robot frame:
        # Y.imu -> X.robot; Z.imu -> Y.robot; X.imu -> Z.robot
        def rotate_frame(x, y, z):
            """Input: x, y, z in IMU frame
            Output: x', y', z' in robot frame"""
            return y, z, x

        # Convert to numpy arrays and rotate to robot frame
        accel_data = np.array(rotate_frame(*accel_data))
        gyro_data = np.array(rotate_frame(*gyro_data))

        if mag_data is None:
            mag_data = np.array([0, 0, 0])
        else:
            mag_data = np.array(mag_data)

        # Update gyro data with offset
        corrected_gyro = self.offset.update(gyro_data)

        # Update the AHRS algorithm with the new data
        self.ahrs.update(corrected_gyro, accel_data, mag_data, delta_time)

        # Obtain the Euler angles from the quaternion
        euler_angles = self.ahrs.quaternion.to_euler()

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

        return euler_angles, internal_states, flags
