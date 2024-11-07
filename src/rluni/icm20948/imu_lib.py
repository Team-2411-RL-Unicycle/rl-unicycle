import logging
import struct
import time

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
    CHIP_ID = 0xEA     # Id for generic ICM-20948 device from the spec sheet
    WRITE_REG_DELAY = 0.0001 # 100us delay after writing a register

    def __init__(
        self, i2c_addr=0x69, i2c_bus=1, accel_range=2, gyro_range=500, config_file=None
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

        logger.info("Configuration parsed successfully.")

    def initialize(self):
        """Initialize the device"""
        # Power and user configuration
        self.select_register_bank(0)
        self.write(self.reg.PWR_MGMT_1, 0b10000000)  # Device Reset command
        time.sleep(0.01)
        self.write(self.reg.PWR_MGMT_1, 0x01)  # Auto select best clock source
        self.write(self.reg.PWR_MGMT_2, 0x00)  # Enable Accel and Gyro (Turned On)
        self.write(self.reg.USER_CTRL, 0b00100000) # Enable I2C Master mode

        # The WhoAmI register contains the device ID for the IMU model, this verifies correct device connected.
        who_am_i = self.read(self.reg.WHO_AM_I)
        if who_am_i == self.CHIP_ID:
            logger.info("ICM-20948 is online!")
        else:
            raise RuntimeError(
                "ICM-20948 initialization failed: WHO_AM_I mismatch. "
                "Different device ID returned than expected."
            )

        # Set range and low pass filtering for accel and gyro
        self.set_accel_range(self._accel_range)
        self.set_accel_LPF(dlpf_cfg=self._accel_LPF_CFG, enable=self._accel_LPF)
        self.set_accel_samplerate(self._accel_rate)
        self.set_gyro_range(self._gyro_range)
        self.set_gyro_LPF(dlpf_cfg=self._gyro_LPF_CFG, enable=self._gyro_LPF)
        
        self.configure_magnetometer()

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
        """ Sets the accelerometer sample rate in Hz """
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
        """ Sets the gyro sample rate in Hz """
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

    def convert_accel(self, raw):
        """Convert raw accelerometer data to G's"""
        return raw * (self._accel_range / 32768)

    def convert_gyro(self, raw):
        """Convert raw gyroscope data to degrees per second"""
        return raw * (self._gyro_range / 32768)
    
    def read_magnetometer(self):
        """Read magnetometer data and return it as a tuple of x, y, z values in μT"""
        self.select_register_bank(0)
        data = self.read_bytes(self.reg.EXT_SLV_SENS_DATA_00, 7)  # Read only magnetometer data registers
        logger.debug(f"Magnetometer raw data: {[hex(x) for x in data]}")

        # Unpack little-endian signed shorts
        mx, my, mz = struct.unpack('<hhh', bytes(data[0:6]))
        logger.debug(f"Raw magnetometer readings: mx={mx}, my={my}, mz={mz}")

        # Scale factor for AK09916 is 0.15 μT per LSB
        mx_uT = mx * 0.15
        my_uT = my * 0.15
        mz_uT = mz * 0.15

        return mx_uT, my_uT, mz_uT
    
    """ Magnetometer Functions """
    def configure_magnetometer(self):
        """Initialize and configure the magnetometer"""
        self.i2c_master_reset()
        self.i2c_master_passthrough(False)
        self.i2c_master_enable(True)
        
        # Reset magnetometer
        self.i2c_master_single_w(self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_CNTL3, 0x01)
        time.sleep(0.01)

        # Reset I2C master if needed
        for _ in range(5):
            if self.mag_who_am_i():
                logger.debug("Magnetometer connected.")
                break
            self.i2c_master_reset()
            time.sleep(0.1)
        else:
            raise RuntimeError("Failed to connect to magnetometer.")

        # Set magnetometer to continuous measurement mode at 100Hz
        self.i2c_master_single_w(self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_CNTL2, 0x08)
        time.sleep(0.01)  # Allow time for the magnetometer to initialize
        
        # Set I2C Master ODR to 100 Hz
        self.configure_i2c_master_odr(100)
        time.sleep(0.01)

        # Configure ICM20948's I2C master to read from ST1 to ST2 registers
        self.i2c_master_configure_slave(self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_ST1, 9)

    def monitor_mag_data_ready(self, iterations=10, delay=0.1):
        """Monitor the magnetometer ST1 register to check if data becomes ready."""
        for i in range(iterations):
            st1 = self.i2c_master_single_r(self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_ST1)
            logger.debug(f"[{i}] Magnetometer ST1 register: 0x{st1:02X}")
            if st1 & 0x01:
                logger.info(f"Data ready at iteration {i}")
            else:
                logger.info(f"Data not ready at iteration {i}")
            time.sleep(delay)

    
    def i2c_master_passthrough(self, enable):
        """Enable or disable I2C master passthrough"""
        self.select_register_bank(0)
        reg_val = self.read(self.reg.INT_PIN_CFG)
        if enable:
            reg_val |= (1 << 1)  # Set BYPASS_EN bit [1]
        else:
            reg_val &= ~(1 << 1)  # Clear BYPASS_EN bit [1]
        self.write(self.reg.INT_PIN_CFG, reg_val)
        
    def i2c_master_enable(self, enable):
        """Enable or disable I2C master"""
        self.i2c_master_passthrough(False)  # Ensure BYPASS_EN is disabled
        time.sleep(.005)
        
        self.select_register_bank(3)
        reg_val = self.read(self.reg.I2C_MST_CTRL)
        reg_val &= ~(0x0F)  # Clear I2C_MST_CLK bits [3:0]
        reg_val |= 0x07  # Set I2C master clock to 345.6 kHz
        reg_val |= (1 << 4)  # Set I2C_MST_P_NSR bit [4]
        self.write(self.reg.I2C_MST_CTRL, reg_val)
        time.sleep(.005)
        
        self.select_register_bank(0)
        reg_val = self.read(self.reg.USER_CTRL)
        if enable:
            reg_val |= (1 << 5)  # Set I2C_MST_EN bit [5]
        else:
            reg_val &= ~(1 << 5)  # Clear I2C_MST_EN bit [5]
        self.write(self.reg.USER_CTRL, reg_val)
        time.sleep(.005)
                
    def i2c_master_reset(self):
        """Reset I2C master module"""
        self.select_register_bank(0)
        reg_val = self.read(self.reg.USER_CTRL)
        reg_val |= (1 << 1)  # Set I2C_MST_RST bit [1]
        self.write(self.reg.USER_CTRL, reg_val)
        
    def configure_i2c_master_odr(self, odr):
        """Configure I2C Master ODR (Output Data Rate) to match magnetometer's ODR."""
        self.select_register_bank(3)
        odr_values = {
            0.95: 0x00,
            12: 0x01,
            25: 0x02,
            50: 0x03,
            100: 0x04,
            200: 0x05,
            400: 0x06,
            800: 0x07,
            1000: 0x08,
        }
        if odr not in odr_values:
            raise ValueError("Invalid ODR value. Valid options are: " + ", ".join(map(str, odr_values.keys())))
        reg_val = odr_values[odr]
        self.write(self.reg.I2C_MST_ODR_CONFIG, reg_val)
        logger.debug(f"I2C Master ODR set to {odr} Hz")
        self.select_register_bank(0)
        
    def i2c_master_single_w(self, addr, reg, data):
        """Write a single byte to an I2C slave device via the ICM20948's I2C master"""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV4_ADDR, addr & 0x7F)  # Write operation
        self.write(self.reg.I2C_SLV4_REG, reg)
        self.write(self.reg.I2C_SLV4_DO, data)
        self.write(self.reg.I2C_SLV4_CTRL, 0x80)  # Enable transaction

        # Wait for completion
        for _ in range(1000):
            self.select_register_bank(0)
            status = self.read(self.reg.I2C_MST_STATUS)
            if status & (1 << 6):
                break
            time.sleep(0.001)
        else:
            raise RuntimeError("I2C master write transaction timed out.")
        
    def i2c_master_single_r(self, addr, reg):
        """Read a single byte from an I2C slave device via the ICM20948's I2C master"""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV4_ADDR, addr | 0x80)  # Read operation
        self.write(self.reg.I2C_SLV4_REG, reg)
        self.write(self.reg.I2C_SLV4_CTRL, 0x80)  # Enable transaction

        # Wait for completion
        for _ in range(1000):
            self.select_register_bank(0)
            status = self.read(self.reg.I2C_MST_STATUS)
            if status & (1 << 6):
                break
            time.sleep(0.001)
        else:
            raise RuntimeError("I2C master read transaction timed out.")

        self.select_register_bank(3)
        return self.read(self.reg.I2C_SLV4_DI)
    
    def mag_who_am_i(self):
        """Verify magnetometer connection"""
        who_am_i = self.i2c_master_single_r(self.reg.AK09916_I2C_ADDR, self.reg.MAG_REG_WIA)
        return who_am_i == 0x09

    def i2c_master_configure_slave(self, addr, reg, length):
        """Configure I2C Master to read data from an I2C slave device via Slave0 with WAIT_FOR_ES."""
        self.select_register_bank(3)
        self.write(self.reg.I2C_SLV0_ADDR, addr | 0x80)  # Read operation
        self.write(self.reg.I2C_SLV0_REG, reg)
        # Set WAIT_FOR_ES (bit 6) and ENABLE (bit 7)
        self.write(self.reg.I2C_SLV0_CTRL, 0b10000000 | length)  # 0xC0 = 0b11000000
        logger.debug("I2C Master configured with WAIT_FOR_ES enabled")

        
    def _twos_complement(self, val, bits):
        """Compute the 2's complement of int value val"""
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val


    def close(self):
        """Clean up"""
        self._bus.close()
