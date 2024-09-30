import yaml
import logging
import os
import struct
import time

import smbus2

try:
    from . import icm20948_registers
except ImportError:
    import icm20948_registers

# Create a logger for your module
logger = logging.getLogger(__name__)

"""
The ICM-20948 is a 9-axis IMU Library for I2C communication with the Jetson Nano

"""


class ICM20948:
    # Id for generic ICM-20948 device
    CHIP_ID = 0xEA
    WRITE_REG_DELAY = 0.0001

    def __init__(
        self, i2c_addr=0x69, i2c_bus=1, accel_range=2, gyro_range=500, config_file=None
    ):
        # Import a register map for the ICM-20948
        self.reg = icm20948_registers.ICM20948Registers()
        # Set the device address and bus number
        self._bus = smbus2.SMBus(i2c_bus)
        self._addr = i2c_addr
        self._bank = 0
        self._accel_range = accel_range
        self._accel_LPF = False  # Low pass filter for accelerometer
        self._accel_LPF_CFG = 0b010  # Default low pass filter configuration
        self._gyro_range = gyro_range
        self._gyro_LPF = False  # Low pass filter for gyroscope
        self._gyro_LPF_CFG = 0b010  # Default low pass filter configuration

        self._accel_bias = [0, 0, 0]
        self._gyro_bias = [0, 0, 0]

        if config_file != None:
            self.load_config(config_file)
            pass
        self.initialize()

    def load_config(self, config_file):
        logger.debug(f"Loading config from: {config_file}")

        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
            logger.debug("Configuration file loaded.")
        except FileNotFoundError:
            logger.error(f"Configuration file not found: {config_file}")
            return
        except yaml.YAMLError as e:
            logger.error(f"Error parsing YAML file {config_file}: {e}")
            return

        # Parse ICM20948 settings
        icm_config = config.get('ICM20948_Configuration', {})
        self._addr = self._get_value(icm_config, 'i2c_addr', self._addr, int)
        self._bus = smbus2.SMBus(self._get_value(icm_config, 'i2c_bus', self._bus, int))
        self._accel_range = self._get_value(icm_config, 'accel_range', self._accel_range, int)
        
        # Directly handle booleans
        self._accel_LPF = self._get_value(icm_config, 'accel_LPF', False, bool)
        self._accel_LPF_CFG = self._get_value(icm_config, 'accel_LPF_CFG', self._accel_LPF_CFG, int)
        self._gyro_range = self._get_value(icm_config, 'gyro_range', self._gyro_range, int)
        self._gyro_LPF = self._get_value(icm_config, 'gyro_LPF', True, bool)
        self._gyro_LPF_CFG = self._get_value(icm_config, 'gyro_LPF_CFG', self._gyro_LPF_CFG, int)

        # Parse calibration data
        calibration = config.get('Calibration', {})
        self._accel_bias = self._get_value(calibration, 'accel_bias', self._accel_bias, list)
        self._gyro_bias = self._get_value(calibration, 'gyro_bias', self._gyro_bias, list)

        logger.info("Configuration parsed successfully.")

    def _get_value(self, config_section, key, default, expected_type=None):
        """
        Helper method to retrieve a configuration value, with optional type validation.
        If the key is not found, it returns the default value.
        """
        value = config_section.get(key, default)
        if expected_type and not isinstance(value, expected_type):
            logger.warning(f"Config key '{key}' expected type {expected_type.__name__} but got {type(value).__name__}. Using default: {default}")
            return default
        return value

    def initialize(self):
        """Initialize the device"""
        # Power and user configuration
        self.write(self.reg.REG_BANK_SEL, 0x00)  # Set to user bank
        time.sleep(0.1)
        self.write(self.reg.PWR_MGMT_1, 0b10000000)  # Device Reset command
        time.sleep(0.1)
        self.write(self.reg.USER_CTRL, 0b00110000)
        time.sleep(0.1)
        self.write(self.reg.PWR_MGMT_1, 0x01)  # Auto select best clock source
        time.sleep(0.1)

        # The WhoAmI register contains the device ID for the IMU model, this verifies correct device connected.
        who_am_i = self.read(self.reg.WHO_AM_I)
        if who_am_i == self.CHIP_ID:
            logger.info("ICM-20948 is online!")
        else:
            raise RuntimeError(
                "ICM-20948 initialization failed: WHO_AM_I mismatch. "
                "Different device ID returned than expected."
            )

        # Set Accel and Gyro
        self.write(self.reg.PWR_MGMT_2, 0x00)  # Enable Accel and Gyro (Turned On)
        # Set range and low pass filtering for accel and gyro
        self.set_accel_range(self._accel_range)
        self.set_accel_LPF(dlpf_cfg=self._accel_LPF_CFG, enable=self._accel_LPF)
        self.set_gyro_range(self._gyro_range)
        self.set_gyro_LPF(dlpf_cfg=self._gyro_LPF_CFG, enable=self._gyro_LPF)

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
            self._bank = bank
            # Value is shifted to the 5:4 bits from the register
            new_val = bank << 4
            # Write the new value back to the register
            self.write(self.reg.REG_BANK_SEL, new_val)
            logger.debug(
                f"Register Bank Changed to: 0x{self.read(self.reg.REG_BANK_SEL):02X}"
            )

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

    def convert_accel(self, raw):
        """Convert raw accelerometer data to G's"""
        return raw * (self._accel_range / 32768)

    def convert_gyro(self, raw):
        """Convert raw gyroscope data to degrees per second"""
        return raw * (self._gyro_range / 32768)

    def close(self):
        """Clean up"""
        self._bus.close()


if __name__ == "__main__":
    try:

        imu = ICM20948(config_file="ICM20948_default.ini")

        while True:
            accel_x, accel_y, accel_z, gx, gy, gz = imu.read_accelerometer_gyro(
                convert=True
            )

            print(
                f"Accel|>  X: {accel_x:.6f}, Y: {accel_y:.6f}, Z: {accel_z:.6f}, <Gyro|> X: {gx:.6f}, Y: {gy:.6f}, Z: {gz:.6f}"
            )

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Program exited")
    finally:
        imu.close()
