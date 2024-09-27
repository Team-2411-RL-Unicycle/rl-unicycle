# imu_test.py
import time

from .imu_lib import ICM20948


def test_register_bank_selection_speed(imu: ICM20948, num_iterations: int):
    start_time = time.time()  # Start timing

    for _ in range(num_iterations):
        imu.select_register_bank(0)  # Attempt to set register bank to 0

    end_time = time.time()  # End timing
    duration = end_time - start_time  # Calculate duration
    return duration


def main():
    imu = ICM20948(i2c_addr=0x69, i2c_bus=1, accel_range=2, gyro_range=250)

    # Run the register bank selection speed test
    test_iterations = 1000
    duration = test_register_bank_selection_speed(imu, test_iterations)
    print(
        f"Average time taken to select register bank 0 for {test_iterations} times: {duration/test_iterations*1e6} micro seconds"
    )
    imu.close()


if __name__ == "__main__":
    main()
