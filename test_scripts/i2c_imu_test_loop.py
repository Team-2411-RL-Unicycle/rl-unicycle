import sys, os
import asyncio
import time
import numpy as np
import LoopTimer as LoopTimer
import matplotlib.pyplot as plt

async def read_imu_data(imu):  # 100 Hz control loop  # Initialize the IMU
    return imu.read_accelerometer_gyro() # Read the IMU data

async def control_loop(imu, num_iterations=100):
    LOOP_TIME = 0.01  # 100 Hz control loop
    N_READS = 3  # Number of reads per iteration
    
    # Preallocate memory for the start times and imu data
    start_times = np.zeros(num_iterations)  
    imu_data_array = np.zeros((6, num_iterations*N_READS))  # 6 rows for ax, ay, az, gx, gy, gz
    
        
    for i in range(num_iterations):
        loop_start_time = time.time()  # Record the start time of the iteration
        start_times[i] = loop_start_time  # Store the start time
           
        # Perform n reads of i2c data
        for j in range (N_READS):
            # Read IMU data
            imu_data_array[:, i*N_READS+j]= imu.read_accelerometer_gyro(convert = True)  # Get IMU data
            await asyncio.sleep(0.00001)  # Non-blocking sleep 
     
        # Busy-wait until 10ms have passed since loop_start_time
        # ! This configuration is important to have steady loop periods 
        while (time.time() - loop_start_time) < LOOP_TIME:
            pass  # Busy wait
                    
    return start_times, imu_data_array  # Return the list of iteration durations


def plot_times(execution_times): 
    # Create the histogram
    plt.hist(execution_times, bins=20, color='blue', edgecolor='black')
    plt.title('Histogram of Loop Execution Times')
    plt.xlabel('Execution Time (seconds)')
    plt.ylabel('Frequency')
    
    # The x-axis should reflect the expected execution times range around 10ms
    plt.xlim([min(execution_times), max(execution_times)])
    
    # Set y-axis to log scale
    # plt.yscale('log')

    # Get the directory where the script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Save the histogram
    file = 'test_data/histogram_loop_times.png'
    # Construct the absolute file path
    file_path = os.path.join(script_dir, file)
    # Ensure the directory exists
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    plt.savefig(file_path)
    
def plot_sensor_data(imu_data_array):
    # imu_data_array is expected to be of shape (6, num_samples)
    # where rows are ax, ay, az, gx, gy, gz
    # Get the directory where the script is located
    
    # Accelerometer data plot
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)  # 2 rows, 1 column, 1st subplot
    plt.plot(imu_data_array[0, :], label='Accel X')
    plt.plot(imu_data_array[1, :], label='Accel Y')
    plt.plot(imu_data_array[2, :], label='Accel Z')
    plt.title('Accelerometer Data Over Time')
    plt.xlabel('Sample Number')
    plt.ylabel('Acceleration (g)')
    plt.legend()
    plt.tight_layout()
    # file = 'test_data/accelerometer_data.png'
    # # Construct the absolute file path
    # file_path = os.path.join(script_dir, file)
    # # Ensure the directory exists
    # os.makedirs(os.path.dirname(file_path), exist_ok=True)
    # plt.savefig(file_path)

    # Gyroscope data plot
    plt.subplot(2, 1, 2)  # 2 rows, 1 column, 2nd subplot
    plt.plot(imu_data_array[3, :], label='Gyro X')
    plt.plot(imu_data_array[4, :], label='Gyro Y')
    plt.plot(imu_data_array[5, :], label='Gyro Z')
    plt.title('Gyroscope Data Over Time')
    plt.xlabel('Sample Number')
    plt.ylabel('Rotation Rate (raw)')
    plt.legend()
    plt.tight_layout()
    file = 'test_data/accel_gyro_data.png'
    
    # Get directory path relative to this python file location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Construct the absolute file path
    file_path = os.path.join(script_dir, file)
    # Ensure the directory exists
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    plt.savefig(file_path)
    
    plt.show()


def main():
    # Get the current directory of the script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Add the parent directory to the Python module search path
    parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
    sys.path.append(parent_dir)
    # Now you can perform relative imports
    from icm20948.lib.imu_lib import ICM20948
    
    # I2C Device ID for the IMU on the Jetson
    imu_addr = 0x69
    # Bus 3/5 pins on Jetson (Bus 1)
    i2c_bus = 1
    # Init the IMU from library
    imu = ICM20948(config_file = 'ICM20948_default.ini')

    # Run the control loop
    loop = asyncio.get_event_loop()
    num_iterations = 10000
    start_times, imu_data = loop.run_until_complete(control_loop(imu, num_iterations=num_iterations)) 
    execution_times = np.diff(start_times)  # Calculate the time between each iteration
    loop.close()
    # print(execution_times)
    # Access execution times for analysis
    plot_times(execution_times)
    plot_sensor_data(imu_data)

    # Print average loop time
    print(f"Average loop time: {sum(execution_times)/len(execution_times)} seconds")

    imu.close()  # Close the I2C connection


if __name__ == '__main__':
    main()