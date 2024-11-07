import time
import logging
from rluni.icm20948.imu_lib import ICM20948
from rluni.fusion.AHRSfusion import AHRSfusion
import importlib.resources as pkg_resources

# Configure logging to display debug messages
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def estimate_read_speed(imu: ICM20948, num_reads=1000):

    logger.info(f"Starting I²C speed test with {num_reads} read operations...")
    
    # Init reads to load Python lib
    for _ in range(10):
        imu.read_accelerometer_gyro()

    start_time = time.time()
    for _ in range(num_reads):
        # About 120bits total required to be read
        imu.read_accelerometer_gyro()
    end_time = time.time()

    elapsed_time = end_time - start_time
    average_time = elapsed_time / num_reads
    
    logger.info(f"Estimated read speed: {average_time:.6f} seconds per read operation.")
    return average_time

def test_accel_and_gyro_ranges(imu: ICM20948):
    logger.info("Testing accelerometer and gyroscope ranges...")
    start = time.time()
    read = imu.read_accelerometer_gyro(convert=True)
    mx, my, mz = imu.read_magnetometer()
    end = time.time()
    logger.info(f"Read time: {end-start:.6f} seconds.")
    logger.info(f"Accelerometer: {read[0:3]}")
    logger.info(f"Gyroscope: {read[3:6]}")
    logger.info(f"Magnetometer: {mx, my, mz}")
    

def testing_area(imu : ICM20948):
    res = imu.mag_who_am_i()
    print(f"Magnetometer WHO_AM_I: {res}")
    imu.select_register_bank(2)
    val1 = imu.read(imu.reg.ACCEL_SMPLRT_DIV_1)
    val2 = imu.read(imu.reg.ACCEL_SMPLRT_DIV_2)
    
    print(f"ACCEL_SMPLRT_DIV_1: {val1}")
    print(f"ACCEL_SMPLRT_DIV_2: {val2}")
        
    
    # for _ in range(100):
    #     test_accel_and_gyro_ranges(imu)
    #     time.sleep(0.01)
    # pass
    
if __name__ == "__main__":
    try:
        # Get the full path to the default.yaml file in rluni.configs.imu
        with pkg_resources.path('rluni.configs.imu', 'default.yaml') as config_file_path:
            imu_config_path = str(config_file_path)
        
        # Create an instance of the ICM20948 class
        imu = ICM20948(config_file=imu_config_path)               
        logger.info("IMU Initialized Successfully.")        
        testing_area(imu)


    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        imu.close()
        logger.info("IMU connection closed.")
