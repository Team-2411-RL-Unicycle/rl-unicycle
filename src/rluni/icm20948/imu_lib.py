import importlib.resources as pkg_resources
import logging
import struct
import time

import numpy as np
import smbus2

try:
    from . import icm20948_registers
except ImportError:
    import icm20948_registers

from rluni.utils import get_validated_config_value as gvcv
from rluni.utils import load_config_file

# Create a logger for your module
logger = logging.getLogger(__name__)

"""
The ICM-20948 is a 9-axis IMU Library for I2C communication with the Jetson Nano

"""


class ICM20948:
    CHIP_ID = 0xEA  # Id for generic ICM-20948 device from the spec sheet
    MAG_ID = 0x09  # Id for the AK09916 magnetometer from the spec sheet
    WRITE_REG_DELAY = 0.0001  # 100us delay after writing a register
    NUM_MAG_READ_BYTES = (
        8  # Number of sequential bytes to continuously read from the magnetometer
    )

    def __init__(
        self,
        i2c_addr=0x69,
        i2c_bus: int = 1,
        accel_range: int = 2,
        gyro_range: int = 500,
        enable_mag: bool = True,
        config_file=None,
    ):
        # Import a register map for the ICM-20948
        self.reg = icm20948_registers.ICM20948Registers()
        # Set the device address and bus number
        self._bus = smbus2.SMBus(i2c_bus)
        self._addr = i2c_addr
        self._bank = None
        self._accel_range = accel_range
        self._accel_LPF = False  # Low pass filter for accelerometer
        self._accel_LPF_CFG = 0b010  # Default low pass filter configuration
        self._accel_rate = 500  # Accelerometer sample rate in Hz
        self._gyro_range = gyro_range
        self._gyro_LPF = False  # Low pass filter for gyroscope
        self._gyro_LPF_CFG = 0b010  # Default low pass filter configuration
        self._gyro_rate = 500  # Gyroscope sample rate in Hz

        self._accel_bias = [0, 0, 0]
        self._gyro_bias = [0, 0, 0]

        self._enable_mag = enable_mag
        # Calibration data
        self.A_1 = np.eye(3)
        self.b = np.zeros([3, 1])

        self._mag_calibration_file = None

        if config_file != None:
            self.config = load_config_file(config_file)
            self._load_config()
        self.initialize()

    def _load_config(self):
        """Load the configuration from a YAML file."""

        # Validate and parse ICM20948 settings
        icm_config = self.config.get("ICM20948_Configuration", {})

        self._addr = gvcv(icm_config, "i2c_addr", int, required=True)
        self._bus = smbus2.SMBus(gvcv(icm_config, "i2c_bus", int, required=True))
        self._accel_range = gvcv(icm_config, "accel_range", int, required=True)

        # Directly handle booleans
        self._accel_LPF = gvcv(icm_config, "accel_LPF", bool, required=True)
        self._accel_LPF_CFG = gvcv(icm_config, "accel_LPF_CFG", int, required=True)
        self._gyro_range = gvcv(icm_config, "gyro_range", int, required=True)
        self._gyro_LPF = gvcv(icm_config, "gyro_LPF", bool, required=True)
        self._gyro_LPF_CFG = gvcv(icm_config, "gyro_LPF_CFG", int, required=True)

        # Parse calibration data
        self._accel_bias = gvcv(
            self.config, "Calibration.accel_bias", list, required=True
        )
        self._gyro_bias = gvcv(
            self.config, "Calibration.gyro_bias", list, required=True
        )

        # Magnetometer settings
        self._enable_mag = gvcv(icm_config, "mag_enable", bool, required=True)

        if self._enable_mag:
            # Check for A, b calibration data and load if available
            self._mag_calibration_file = gvcv(
                self.config, "Calibration.magnetometer", str, required=False
            )
            if self._mag_calibration_file is not None:
                self.load_calibration()

        logger.info("Configuration parsed successfully.")

    def initialize(self):
        """Initialize the device"""
        self.icm_reset(overwrite=True)
        time.sleep(0.01)
        self.verify_device_id()

        # Anable master mode and reset slave
        self.write(self.reg.USER_CTRL, 0b00110000)
        time.sleep(0.01)

        # Auto select best clock source
        self.write(self.reg.PWR_MGMT_1, 0x01)
        time.sleep(0.01)

        # Set Accel and Gyro on
        self.write(self.reg.PWR_MGMT_2, 0x00)
        time.sleep(0.01)

        # Set range and low pass filtering for accel and gyro
        self.set_accel_range(self._accel_range)
        self.set_accel_LPF(dlpf_cfg=self._accel_LPF_CFG, enable=self._accel_LPF)
        time.sleep(0.01)

        self.set_gyro_range(self._gyro_range)
        self.set_gyro_LPF(dlpf_cfg=self._gyro_LPF_CFG, enable=self._gyro_LPF)
        time.sleep(0.01)

        # TODO: Understand implications of setting samplerate, RWIP did not set any
        # self.set_accel_samplerate(self._accel_rate)
        # self.set_gyro_samplerate(self._gyro_rate)

        if self._enable_mag:
            self.configure_magnetometer()

    def icm_reset(self, overwrite=False):
        """Reset the ICM20948 device"""
        self.select_register_bank(0)
        curr_reg = self.read(self.reg.PWR_MGMT_1) if not overwrite else 0x00
        new_reg = curr_reg | (1 << 7)  # Set bit 7 to reset
        self.write(self.reg.PWR_MGMT_1, new_reg)
        time.sleep(0.01)

    def verify_device_id(self):
        # Verify device ID is actually the ICM-20948
        who_am_i = self.read(self.reg.WHO_AM_I)
        if who_am_i == self.CHIP_ID:
            logger.info("ICM-20948 is online!")
        else:
            raise RuntimeError("ICM-20948 initialization failed: WHO_AM_I mismatch. ")

    def read(self, reg):
        """Read a single byte from a register"""
        return self._bus.read_byte_data(self._addr, reg)

    def write(self, reg, data):
        """Write a single byte to a register"""
        self._bus.write_byte_data(self._addr, reg, data)
        time.sleep(self.WRITE_REG_DELAY)

    def read_bytes(self, reg, length):
        """Read a number of bytes from a register"""
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def select_register_bank(self, bank):
        """Select a register bank"""
        if bank != self._bank:
            if bank > 3:
                raise ValueError("Bank must be in the range 0-3")
            self.write(self.reg.REG_BANK_SEL, bank << 4)
            self._bank = bank

    def set_accel_range(self, accel_range=2):
        """Set the range of the accelerometer
        Bit Map for ACCEL_CONFIG:
        7:6 - Reserved
        5:3 - ACCEL_DLPFCFG[2:0] Accelerometer low pass filter configuration
        2:1 - ACCEL_FS_SEL[1:0]
            Accelerometer Full Scale Select:
            00: ±2g
            01: ±4g
            10: ±8g
            11: ±16g
        0 - Enable digital low pass filter
        """

        if accel_range not in [2, 4, 8, 16]:
            raise ValueError("Accel range must be 2, 4, 8, or 16")
        self._accel_range = accel_range
        logger.debug(f"Setting accel range to {self._accel_range}")

        # Data sheet mapping of the ACCEL_FS_SEL bits
        fs_sel_values = {
            2: 0b00,  # ±2g
            4: 0b01,  # ±4g
            8: 0b10,  # ±8g,
            16: 0b11,  # ±16g
        }
        fs_sel = fs_sel_values[accel_range]

        try:
            self.select_register_bank(2)
            # Read the current value of the ACCEL_CONFIG register
            reg_read = self.read(self.reg.ACCEL_CONFIG)
            # Clear the ACCEL_FS_SEL bits (2:1) and set them according to the calculated value
            # Also, set the LSB based on the _accel_LPF flag
            reg_val = (reg_read & 0b11111001) | (fs_sel << 1)
            logger.debug(f"Setting accel range register to {reg_val:02X}")

            # Write the new configuration back to the ACCEL_CONFIG register
            self.write(self.reg.ACCEL_CONFIG, reg_val)
        except Exception as e:
            logger.error(f"Error setting accel range: {e}")

    def set_accel_LPF(self, dlpf_cfg=0, enable=True):
        """Enable or disable the accelerometer low pass filter and set its configuration.

        Parameters:
        dlpf_cfg (int): DLPF configuration setting (0-7).
        enable (bool): Flag to enable (True) or disable (False) the DLPF.
        """

        if dlpf_cfg not in range(8):
            raise ValueError("DLPF configuration must be an integer between 0 and 7")

        self._accel_LPF = enable
        self._accel_LPF_CFG = dlpf_cfg

        # Table of values for Low Pass Filter Configuration (Page 64 of the datasheet ACCEL_FCHOICE)
        dlpfcfg_vals = {
            0: 246.0,  # Hz
            1: 246.0,  # Hz - Duplicate of 0, as per datasheet
            2: 111.4,  # Hz
            3: 50.4,  # Hz
            4: 23.9,  # Hz
            5: 11.5,  # Hz
            6: 5.7,  # Hz
            7: 473.0,  # Hz - Different rate calculation method
        }

        # Log the DLPF configuration setting
        logger.debug(
            f"Setting accel low pass filter to {dlpfcfg_vals[self._accel_LPF_CFG]} Hz"
            f" with {'ENABLED' if self._accel_LPF else 'DISABLED'} DLPF"
        )

        try:
            self.select_register_bank(2)
            # Read the current value of the ACCEL_CONFIG register
            reg_read = self.read(self.reg.ACCEL_CONFIG)

            # Modify the ACCEL_CONFIG register value based on the DLPF settings
            # Bits 5:3 are used for the ACCEL_DLPFCFG setting
            reg_val = (reg_read & 0b11000111) | (self._accel_LPF_CFG << 3)

            # If DLPF is disabled, ACCEL_FCHOICE bit (bit 0) should be set to 0, and to 1 if enabled
            if not enable:
                reg_val &= 0b11111110  # Clear bit 0 to disable DLPF
            else:
                reg_val |= 0b00000001  # Set bit 0 to enable DLPF

            # Write the new configuration back to the ACCEL_CONFIG register
            self.write(self.reg.ACCEL_CONFIG, reg_val)
            logger.debug(f"ACCEL_CONFIG register set to {reg_val:02X}")
        except Exception as e:
            logger.error(f"Error setting accel low pass filter: {e}")

    def set_accel_samplerate(self, rate):
        """Sets the accelerometer sample rate in Hz"""
        # rate = 1.125khz/(1+ratediv)
        ratediv = int(1125 / rate - 1)
        ratediv = max(0, min(0x0FFF, ratediv))
        self.select_register_bank(2)
        self.write(self.reg.ACCEL_SMPLRT_DIV_1, ratediv & 0xFF)
        self.write(self.reg.ACCEL_SMPLRT_DIV_2, ratediv >> 8)

    def set_gyro_range(self, gyro_range=250):
        if gyro_range not in [250, 500, 1000, 2000]:
            raise ValueError("Gyro range must be 250, 500, 1000, or 2000")
        self._gyro_range = gyro_range
        logger.debug(f"Setting gyro range to {self._gyro_range}")

        # Data sheet mapping of the ACCEL_FS_SEL bits
        fs_sel_values = {
            250: 0b00,  # ±250deg/s
            500: 0b01,  # ±500deg/s
            1000: 0b10,  # ±1000deg/s
            2000: 0b11,  # ±2000deg/s
        }
        fs_sel = fs_sel_values[gyro_range]

        try:
            self.select_register_bank(2)
            # Read the current value of the GYRO_CONFIG register
            reg_read = self.read(self.reg.GYRO_CONFIG_1)
            # Clear the ACCEL_FS_SEL bits (2:1) and set them according to the calculated value
            reg_val = (reg_read & 0b11111001) | (fs_sel << 1)
            logger.debug(f"Setting gyro range register to {reg_val:02X}")

            # Write the new configuration back to the GYRO_CONFIG register
            self.write(self.reg.GYRO_CONFIG_1, reg_val)
        except Exception as e:
            logger.error(f"Error setting gyro range: {e}")

    def set_gyro_LPF(self, dlpf_cfg=0, enable=True):
        """Enable or disable the gyroscope low pass filter and set its configuration.

        Parameters:
        dlpf_cfg (int): DLPF configuration setting (0-7).
        enable (bool): Flag to enable (True) or disable (False) the DLPF.
        """

        if dlpf_cfg not in range(8):
            raise ValueError("DLPF configuration must be an integer between 0 and 7")

        self._gyro_LPF = enable
        self._gyro_LPF_CFG = dlpf_cfg

        # Table of values for Low Pass Filter Configuration (Page 64 of the datasheet GYRO_FCHOICE)
        dlpfcfg_vals = {
            0: 197.0,  # Hz
            1: 151.0,  # Hz
            2: 119.0,  # Hz
            3: 51.0,  # Hz
            4: 23.0,  # Hz
            5: 11.0,  # Hz
            6: 5.0,  # Hz
            7: 361.0,  # Hz - Different rate calculation method
        }

        # Log the DLPF configuration setting
        logger.debug(
            f"Setting gyro low pass filter to {dlpfcfg_vals[self._gyro_LPF_CFG]} Hz "
            f" with {'enabled' if self._gyro_LPF else 'disabled'} DLPF"
        )

        try:
            self.select_register_bank(2)
            # Read the current value of the GYRO_CONFIG_1 register
            reg_read = self.read(self.reg.GYRO_CONFIG_1)

            # Modify the GYRO_CONFIG_1 register value based on the DLPF settings
            # Bits 5:3 are used for the GYRO_DLPFCFG setting
            reg_val = (reg_read & 0b11000111) | (self._gyro_LPF_CFG << 3)

            # If DLPF is disabled, GYRO_FCHOICE bit (bit 0) should be set to 0, and to 1 if enabled
            if not enable:
                reg_val &= 0b11111110  # Clear bit 0 to disable DLPF
            else:
                reg_val |= 0b00000001  # Set bit 0 to enable DLPF

            # Write the new configuration back to the GYRO_CONFIG_1 register
            self.write(self.reg.GYRO_CONFIG_1, reg_val)
            logger.debug(f"GYRO_CONFIG_1 register set to {reg_val:02X}")
        except Exception as e:
            logger.error(f"Error setting gyro low pass filter: {e}")

    def set_gyro_samplerate(self, rate):
        """Sets the gyro sample rate in Hz"""
        # rate = 1.125khz/(1+ratediv)
        ratediv = int(1100 / rate - 1)
        ratediv = max(0, min(0xFF, ratediv))
        self.select_register_bank(2)
        self.write(self.reg.GYRO_SMPLRT_DIV, ratediv)

    def read_accelerometer_gyro(self, convert=False):
        """Read accelerometer data and return it as a tuple of x, y, z values

        Optionally convert the raw data to G's and degrees per second
        """
        self.select_register_bank(0)
        # Read the 12 consecutive state registers for the accelerometer and gyroscope (starts at ACCEL_XOUT_H address)
        data = self.read_bytes(self.reg.ACCEL_XOUT_H, 12)
        # > is big-endian, h is a short int (2 bytes), we have 6 short ints from the sensor to unpack
        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data))

        # Convert to actual readings using range and bias
        if convert:
            ax = self.convert_accel(ax) - self._accel_bias[0]
            ay = self.convert_accel(ay) - self._accel_bias[1]
            az = self.convert_accel(az) - self._accel_bias[2]
            gx = self.convert_gyro(gx) - self._gyro_bias[0]
            gy = self.convert_gyro(gy) - self._gyro_bias[1]
            gz = self.convert_gyro(gz) - self._gyro_bias[2]

        return ax, ay, az, gx, gy, gz

    def read_magnetometer(self, calibrated=True):
        """
        Read magnetometer data after configure magnetometer has been run at startup

        Returns:
            mx_uT (float): X-axis magnetometer reading in μT
            my_uT (float): Y-axis magnetometer reading in μT
            mz_uT (float): Z-axis magnetometer reading in μT
            overflow_flag (int): 1 if magnetic sensor overflow, 0 otherwise
        """

        if not self._enable_mag:
            raise ValueError(
                "Magnetometer read attempted, but it is not enabled in imu settings."
            )

        """Read magnetometer data and return it as a tuple of x, y, z values in μT"""
        self.select_register_bank(0)

        # Pick up mag data cached by the ICM20948 and overflow indicator
        data = self.read_bytes(self.reg.EXT_SLV_SENS_DATA_00, self.NUM_MAG_READ_BYTES)
        # Unpack with 2's complement and little-endian byte order
        mx, my, mz = struct.unpack("<hhh", bytes(data[0:6]))

        # Bit 3 of ST2 indicates magnetic sensor overflow
        overflow_flag = data[-1] & 0x08

        # Convert to actual readings in μT
        mx_uT, my_uT, mz_uT = self.convert_magnetometer(
            mx, my, mz, calibrated=calibrated
        )

        return mx_uT, my_uT, mz_uT, overflow_flag

    def read_agtm(self, convert=True):
        """Read Accelerometer, Gyroscope, Temperature and Magnetometer data in a single I2C read operation"""
        self.select_register_bank(0)

        total_bytes = 6 + 6 + 2 + self.NUM_MAG_READ_BYTES
        data = self.read_bytes(self.reg.ACCEL_XOUT_H, total_bytes)

        # Unpack accelerometer and gyroscope data (big-endian format)
        ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", bytearray(data[0:12]))
        # Unpack temperature data (big-endian format)
        (temp,) = struct.unpack(">h", bytes(data[12:14]))
        # Unpack magetometer data (little-endian format)
        mag_data = data[14:]

        mx, my, mz = struct.unpack("<hhh", bytes(mag_data[0:6]))
        overflow_flag = mag_data[-1] & 0x08

        if convert:
            ax = self.convert_accel(ax) - self._accel_bias[0]
            ay = self.convert_accel(ay) - self._accel_bias[1]
            az = self.convert_accel(az) - self._accel_bias[2]
            gx = self.convert_gyro(gx) - self._gyro_bias[0]
            gy = self.convert_gyro(gy) - self._gyro_bias[1]
            gz = self.convert_gyro(gz) - self._gyro_bias[2]
            temp = self.convert_temp(temp)
            # Rotate into the same frame as the robot
            mx, my, mz = self.convert_magnetometer(mx, my, mz, rotate=True)

        return ax, ay, az, gx, gy, gz, temp, mx, my, mz, overflow_flag

    def read_sensor_data(self, convert=True):
        """Conditional read depending if magnetometer is enabled"""
        if self._enable_mag:
            return self.read_agtm(convert)
        else:
            return self.read_accelerometer_gyro(convert)

    def convert_accel(self, raw):
        """Convert raw accelerometer data to G's"""
        return raw * (self._accel_range / 32768)

    def convert_gyro(self, raw):
        """Convert raw gyroscope data to degrees per second"""
        return raw * (self._gyro_range / 32768)

    def convert_magnetometer(self, mx, my, mz, rotate=True, calibrated=True):
        """Convert magnetomer to same frame as accel and gyro and uT"""

        if rotate:
            mx = mx
            my = -my
            mz = -mz

        mx = mx * 0.15
        my = my * 0.15
        mz = mz * 0.15

        if calibrated:
            s = np.array([mx, my, mz]).reshape(3, 1)
            s = np.dot(self.A_1, s - self.b)
            mx, my, mz = s[0, 0], s[1, 0], s[2, 0]

        return mx, my, mz

    def convert_temp(self, raw_temp):
        """Convert raw temperature to degrees Celsius."""
        # According to the datasheet:
        # Temperature in degrees C = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 21
        # Temp_Sensitivity is typically 333.87 LSB/°C
        # RoomTemp_Offset is device-specific, typically 0 at 21°C
        temp_c = ((raw_temp - 0) / 333.87) + 21
        return temp_c

    """ Continuous I2C functions for peripheral communication with the ICM20948 and Magnetometer via I2C Slave 0"""

    def i2c_master_passthrough(self, enable):
        """Enable or disable I2C master passthrough"""
        self.select_register_bank(0)
        reg_val = self.read(self.reg.INT_PIN_CFG)
        if enable:
            reg_val |= 1 << 1  # Set BYPASS_EN bit [1]
        else:
            reg_val &= ~(1 << 1)  # Clear BYPASS_EN bit [1]
        self.write(self.reg.INT_PIN_CFG, reg_val)

    def i2c_master_enable(self, enable):
        """Enable or disable I2C master"""
        self.i2c_master_passthrough(False)  # Ensure BYPASS_EN is disabled
        time.sleep(0.005)

        self.select_register_bank(3)
        reg_val = self.read(self.reg.I2C_MST_CTRL)
        reg_val &= ~(0x0F)  # Clear I2C_MST_CLK bits [3:0]
        reg_val |= 0x07  # Set I2C master clock to 345.6 kHz
        reg_val |= 1 << 4  # Set I2C_MST_P_NSR bit [4]
        self.write(self.reg.I2C_MST_CTRL, reg_val)
        time.sleep(0.005)

        self.select_register_bank(0)
        reg_val = self.read(self.reg.USER_CTRL)
        if enable:
            reg_val |= 1 << 5  # Set I2C_MST_EN bit [5]
        else:
            reg_val &= ~(1 << 5)  # Clear I2C_MST_EN bit [5]
        self.write(self.reg.USER_CTRL, reg_val)
        time.sleep(0.005)

    def i2c_master_reset(self):
        """Reset I2C master module"""
        self.select_register_bank(0)
        reg_val = self.read(self.reg.USER_CTRL)
        reg_val |= 1 << 1  # Set I2C_MST_RST bit [1]
        self.write(self.reg.USER_CTRL, reg_val)

    def i2c_master_odr_configure(self, odr):
        """Configure I²C Master Output Data Rate to match magnetometer's ODR."""
        odr_values = {
            1.1: 0x00,
            12: 0x01,
            25: 0x02,
            50: 0x03,
            100: 0x04,
            200: 0x05,
            400: 0x06,
            800: 0x07,
        }
        if odr not in odr_values:
            raise ValueError(
                "Invalid ODR value. Valid options are: "
                + ", ".join(map(str, odr_values.keys()))
            )
        self.select_register_bank(3)
        self.write(self.reg.I2C_MST_ODR_CONFIG, odr_values[odr])
        logger.debug(f"I²C Master ODR set to {odr} Hz")

    def i2c_master_configure_slave0_read(self, addr, reg, length):
        """Configure I2C Master to read data from an I2C slave device via Slave0"""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV0_ADDR, addr | 0x80)  # Read operation
        self.write(self.reg.I2C_SLV0_REG, reg)
        # bit 7 = enable reading from slave at sample rate and store to EXT_SENS_DATA_00
        # bit 6 = Swap bytes: this is handled by the struct.unpack method, so 0
        self.write(self.reg.I2C_SLV0_CTRL, 0b10000000 | length)

    """ Magnetometer Functions: Single Read/Write via I2C Slave 4"""

    def configure_magnetometer(self):
        """Initialize and configure the magnetometer"""
        # Enable I2C master
        self.i2c_master_enable(True)
        # Reset magnetometer
        self.mag_reset()
        self.mag_test_sequence()
        # Set magnetometer to continuous measurement mode at 100Hz
        self.mag_set_mode(0x08)
        # Set I2C Master output datarate to 100 Hz
        self.i2c_master_odr_configure(100)
        time.sleep(0.01)

        # Configure ICM20948's I2C master to read all mag data and terminating ST2 register (8 bytes)
        self.i2c_master_configure_slave0_read(
            self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_HXL, self.NUM_MAG_READ_BYTES
        )

    def mag_check_id(self):
        # Check connection established with magnetometer
        for _ in range(5):
            who_am_i = self.mag_read(self.reg.MAG_REG_WIA)
            if who_am_i == self.MAG_ID:
                logger.debug("Magnetometer connected.")
                break
            self.i2c_master_reset()
            time.sleep(0.01)
        else:
            raise RuntimeError("Failed to connect to magnetometer.")

    def mag_write(self, reg, data):
        """Write a single byte to magnetometer via I2C4"""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV4_ADDR, self.reg.AK09916_I2C_ADDR & 0x7F)
        self.write(self.reg.I2C_SLV4_REG, reg)
        self.write(self.reg.I2C_SLV4_DO, data)
        self.write(self.reg.I2C_SLV4_CTRL, 0x80)  # Enable transaction
        self.mag_io_wait()

    def mag_read(self, reg):
        """Read a single byte from the magnetometer via I2C4"""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV4_ADDR, self.reg.AK09916_I2C_ADDR | 0x80)
        self.write(self.reg.I2C_SLV4_REG, reg)
        self.write(self.reg.I2C_SLV4_CTRL, 0x80)  # Enable transaction
        self.mag_io_wait()

        self.select_register_bank(3)
        return self.read(self.reg.I2C_SLV4_DI)

    def mag_io_wait(self):
        """Wait for magnetometer data to be ready"""
        for _ in range(1000):
            self.select_register_bank(0)
            status = self.read(self.reg.I2C_MST_STATUS)
            if status & (1 << 6):
                break
            time.sleep(0.001)
        else:
            raise RuntimeError("I2C master I/O wait timed out.")

    def mag_reset(self):
        self.mag_write(self.reg.MAG_REG_CNTL3, 0x01)
        time.sleep(0.01)

    def mag_set_mode(self, mode):
        """Set the magnetometer mode"""

        mode_values = {
            0x00: "power_down",
            0x01: "single_measurement",
            0x02: "continuous_10Hz",
            0x04: "continuous_20Hz",
            0x06: "continuous_50Hz",
            0x08: "continuous_100Hz",
            0x10: "self_test",
        }

        if mode not in mode_values:
            raise ValueError(
                "Invalid magnetometer mode. Valid options are: "
                + ", ".join(f"0x{val:02X}" for val in mode_values.keys())
            )

        mode_name = mode_values[mode]
        logger.debug(f"Setting magnetometer mode to: {mode_name} (0x{mode:02X})")

        self.mag_write(self.reg.MAG_REG_CNTL2, mode)
        time.sleep(0.01)

    def mag_test_sequence(self):
        """ """
        # Set power down mode
        self.mag_set_mode(0x00)
        time.sleep(0.01)

        # Set self-test mode
        self.mag_set_mode(0x10)

        # Wait for data to be ready
        for _ in range(100):
            check_ready = self.mag_read(self.reg.MAG_REG_ST1) & 0x01
            if check_ready:
                break
            time.sleep(0.01)

        if not check_ready:
            raise RuntimeError("Magnetometer self-test failed, unable to read data.")

        # Read HXL to HZH (6 bytes) and unpack them
        data = [
            self.mag_read(i)
            for i in range(self.reg.MAG_REG_HXL, self.reg.MAG_REG_HZH + 1)
        ]
        hx, hy, hz = struct.unpack("<hhh", bytes(data))

        # Define the self-test criteria
        if not (-200 <= hx <= 200):
            raise ValueError(f"Self-test failed for HX: {hx} not in range -200 to 200")
        if not (-200 <= hy <= 200):
            raise ValueError(f"Self-test failed for HY: {hy} not in range -200 to 200")
        if not (-1000 <= hz <= -200):
            raise ValueError(
                f"Self-test failed for HZ: {hz} not in range -1000 to -200"
            )

        self.mag_set_mode(0x00)
        # Log successful self-test
        logger.info("Magnetometer self-test passed successfully.")

    def load_calibration(self):
        """Loads the calibration data (A_1 and b) from a file if available."""
        try:
            with pkg_resources.path(
                "rluni.configs.imu", self._mag_calibration_file
            ) as config_file_path:
                with np.load(config_file_path) as data:
                    self.A_1 = data["A_1"]
                    self.b = data["b"]

            print(f"Loaded calibration data from {self._mag_calibration_file}")
        except FileNotFoundError:
            print("No calibration data file found. Proceeding without calibration.")

    def close(self):
        """Clean up"""
        self._bus.close()
