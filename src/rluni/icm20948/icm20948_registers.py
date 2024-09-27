class ICM20948Registers:
    """Full register map for the ICM-20948 IMU.
    https://invensense.tdk.com/wp-content/uploads/2021/10/DS-000189-ICM-20948-v1.5.pdf
    """

    # ICM-20948 User Bank 0 Register Map
    WHO_AM_I = 0x00  # Device ID
    USER_CTRL = 0x03  # User Control
    LP_CONFIG = 0x05  # Low Power Configuration
    PWR_MGMT_1 = 0x06  # Power Management 1
    PWR_MGMT_2 = 0x07  # Power Management 2
    INT_PIN_CFG = 0x0F  # Interrupt Pin Configuration
    INT_ENABLE = 0x10  # Interrupt Enable
    INT_ENABLE_1 = 0x11  # Interrupt Enable 1
    INT_ENABLE_2 = 0x12  # Interrupt Enable 2
    INT_ENABLE_3 = 0x13  # Interrupt Enable 3
    I2C_MST_STATUS = 0x17  # I2C Master Status
    INT_STATUS = 0x19  # Interrupt Status
    INT_STATUS_1 = 0x1A  # Interrupt Status 1
    INT_STATUS_2 = 0x1B  # Interrupt Status 2
    INT_STATUS_3 = 0x1C  # Interrupt Status 3
    DELAY_TIMEH = 0x28  # Delay Time High
    DELAY_TIMEL = 0x29  # Delay Time Low
    ACCEL_XOUT_H = 0x2D  # Accelerometer X-axis High Byte
    ACCEL_XOUT_L = 0x2E  # Accelerometer X-axis Low Byte
    ACCEL_YOUT_H = 0x2F  # Accelerometer Y-axis High Byte
    ACCEL_YOUT_L = 0x30  # Accelerometer Y-axis Low Byte
    ACCEL_ZOUT_H = 0x31  # Accelerometer Z-axis High Byte
    ACCEL_ZOUT_L = 0x32  # Accelerometer Z-axis Low Byte
    GYRO_XOUT_H = 0x33  # Gyroscope X-axis High Byte
    GYRO_XOUT_L = 0x34  # Gyroscope X-axis Low Byte
    GYRO_YOUT_H = 0x35  # Gyroscope Y-axis High Byte
    GYRO_YOUT_L = 0x36  # Gyroscope Y-axis Low Byte
    GYRO_ZOUT_H = 0x37  # Gyroscope Z-axis High Byte
    GYRO_ZOUT_L = 0x38  # Gyroscope Z-axis Low Byte
    TEMP_OUT_H = 0x39  # Temperature High Byte
    TEMP_OUT_L = 0x3A  # Temperature Low Byte
    EXT_SLV_SENS_DATA_00 = 0x3B  # External Slave Sensor Data 00
    EXT_SLV_SENS_DATA_01 = 0x3C  # External Slave Sensor Data 01
    EXT_SLV_SENS_DATA_02 = 0x3D  # External Slave Sensor Data 02
    EXT_SLV_SENS_DATA_03 = 0x3E  # External Slave Sensor Data 03
    EXT_SLV_SENS_DATA_04 = 0x3F  # External Slave Sensor Data 04
    EXT_SLV_SENS_DATA_05 = 0x40  # External Slave Sensor Data 05
    EXT_SLV_SENS_DATA_06 = 0x41  # External Slave Sensor Data 06
    EXT_SLV_SENS_DATA_07 = 0x42  # External Slave Sensor Data 07
    EXT_SLV_SENS_DATA_08 = 0x43  # External Slave Sensor Data 08
    EXT_SLV_SENS_DATA_09 = 0x44  # External Slave Sensor Data 09
    EXT_SLV_SENS_DATA_10 = 0x45  # External Slave Sensor Data 10
    EXT_SLV_SENS_DATA_11 = 0x46  # External Slave Sensor Data 11
    EXT_SLV_SENS_DATA_12 = 0x47  # External Slave Sensor Data 12
    EXT_SLV_SENS_DATA_13 = 0x48  # External Slave Sensor Data 13
    EXT_SLV_SENS_DATA_14 = 0x49  # External Slave Sensor Data 14
    EXT_SLV_SENS_DATA_15 = 0x4A  # External Slave Sensor Data 15
    EXT_SLV_SENS_DATA_16 = 0x4B  # External Slave Sensor Data 16
    EXT_SLV_SENS_DATA_17 = 0x4C  # External Slave Sensor Data 17
    EXT_SLV_SENS_DATA_18 = 0x4D  # External Slave Sensor Data 18
    EXT_SLV_SENS_DATA_19 = 0x4E  # External Slave Sensor Data 19
    EXT_SLV_SENS_DATA_20 = 0x4F  # External Slave Sensor Data 20
    EXT_SLV_SENS_DATA_21 = 0x50  # External Slave Sensor Data 21
    EXT_SLV_SENS_DATA_22 = 0x51  # External Slave Sensor Data 22
    EXT_SLV_SENS_DATA_23 = 0x52  # External Slave Sensor Data 23
    FIFO_EN_1 = 0x66  # FIFO Enable 1
    FIFO_EN_2 = 0x67  # FIFO Enable 2
    FIFO_RST = 0x68  # FIFO Reset
    FIFO_MODE = 0x69  # FIFO Mode
    FIFO_COUNTH = 0x70  # FIFO Count High
    FIFO_COUNTL = 0x71  # FIFO Count Low
    FIFO_R_W = 0x72  # FIFO Read Write
    DATA_RDY_STATUS = 0x74  # Data Ready Status
    FIFO_CFG = 0x76  # FIFO Configuration
    REG_BANK_SEL = 0x7F  # Register Bank Select

    # ICM-20948 User Bank 1 Register Map
    SELF_TEST_X_GYRO = 0x02  # Self Test X Gyroscope
    SELF_TEST_Y_GYRO = 0x03  # Self Test Y Gyroscope
    SELF_TEST_Z_GYRO = 0x04  # Self Test Z Gyroscope
    SELF_TEST_X_ACCEL = 0x0E  # Self Test X Accelerometer
    SELF_TEST_Y_ACCEL = 0x0F  # Self Test Y Accelerometer
    SELF_TEST_Z_ACCEL = 0x10  # Self Test Z Accelerometer
    XA_OFFS_H = 0x14  # Accelerometer X-axis Offset High Byte
    XA_OFFS_L = 0x15  # Accelerometer X-axis Offset Low Byte
    YA_OFFS_H = 0x17  # Accelerometer Y-axis Offset High Byte
    YA_OFFS_L = 0x18  # Accelerometer Y-axis Offset Low Byte
    ZA_OFFS_H = 0x1A  # Accelerometer Z-axis Offset High Byte
    ZA_OFFS_L = 0x1B  # Accelerometer Z-axis Offset Low Byte

    # ICM-20948 User Bank 2 Register Map
    GYRO_SMPLRT_DIV = 0x00
    GYRO_CONFIG_1 = 0x01
    GYRO_CONFIG_2 = 0x02
    XG_OFFS_USRH = 0x03
    XG_OFFS_USRL = 0x04
    YG_OFFS_USRH = 0x05
    YG_OFFS_USRL = 0x06
    ZG_OFFS_USRH = 0x07
    ZG_OFFS_USRL = 0x08
    ODR_ALIGN_EN = 0x09
    ACCEL_SMPLRT_DIV_1 = 0x10
    ACCEL_SMPLRT_DIV_2 = 0x11
    ACCEL_INTEL_CTRL = 0x12
    ACCEL_WOM_THR = 0x13
    ACCEL_CONFIG = 0x14
    ACCEL_CONFIG_2 = 0x15
    FSYNC_CONFIG = 0x52
    TEMP_CONFIG = 0x53
    MOD_CTRL_USR = 0x54
    REG_BANK_SEL = 0x7F  # This register is repeated across banks for bank selection.

    # ICM-20948 User Bank 3 Register Map
    I2C_MST_ODR_CONFIG = 0x00
    I2C_MST_CTRL = 0x01
    I2C_MST_DELAY_CTRL = 0x02
    I2C_SLV0_ADDR = 0x03
    I2C_SLV0_REG = 0x04
    I2C_SLV0_CTRL = 0x05
    I2C_SLV0_DO = 0x06
    I2C_SLV1_ADDR = 0x07
    I2C_SLV1_REG = 0x08
    I2C_SLV1_CTRL = 0x09
    I2C_SLV1_DO = 0x0A
    I2C_SLV2_ADDR = 0x0B
    I2C_SLV2_REG = 0x0C
    I2C_SLV2_CTRL = 0x0D
    I2C_SLV2_DO = 0x0E
    I2C_SLV3_ADDR = 0x0F
    I2C_SLV3_REG = 0x10
    I2C_SLV3_CTRL = 0x11
    I2C_SLV3_DO = 0x12
    I2C_SLV4_ADDR = 0x13
    I2C_SLV4_REG = 0x14
    I2C_SLV4_CTRL = 0x15
    I2C_SLV4_DO = 0x16
    I2C_SLV4_DI = 0x17
    REG_BANK_SEL = 0x7F  # This register is repeated across banks for bank selection.

    def __init__(self):
        pass
