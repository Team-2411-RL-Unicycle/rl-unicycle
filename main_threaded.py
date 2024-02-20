import sys, os
import asyncio
import time

import threading
import psutil  

import LoopTimer as lt


class RobotSystem:
    LOOP_TIME = 0.01 # 100 Hz control loop period
    def __init__(self):
        # Get the current directory of the script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        sys.path.append(current_dir)
        # Now you can perform relative imports
        from icm20948.lib.imu_lib import ICM20948
        from communication.mqtt import MQTTClient
        # Init the IMU from library Config files are found in icm20948/configs dir and keep the imu settings
        self.imu = ICM20948(config_file = 'ICM20948_default.ini')
        self.mqtt_client = MQTTClient()  # Initialize MQTT communication
        # Motor drivers
        self.xmotor = None 
        self.ymotor = None
        self.zmotor = None
        # Other robot parts
        
        # Cycle counter
        self.itr = int(0)

    async def control_loop(self):
        loop_total_period = self.LOOP_TIME
        self.stopwatch_i2c = lt.LoopTimer(550)
        self.stopwatch_mqtt = lt.LoopTimer(550)
        while True:
            loop_start_time = time.time()

            self.itr += 1  # Increment the cycle counter 

            # //////////////// Sensor readings
            self.stopwatch_i2c.start()
            ax, ay, az, gx, gy, gz = self.imu.get_accel_gyro_data(convert = True)
            self.stopwatch_i2c.stop()
            
            if self.itr % 500 == 0:
                print(f'Average sensor read {self.stopwatch_i2c.average_time()}')
                print(f' Max sensor read {max(self.stopwatch_i2c.get_execution_times())}')
            
            # //////////////// Control logic and motor commands based on sensor data           
            

            # MQTT communication for telemetry
            loop_delay = (loop_total_period - self.LOOP_TIME) * 1e6  # Convert to microseconds
            
            self.mqtt_client.publish_loop_time(loop_delay)            
            if (self.itr % 20) == 0:  # Downsample 
                self.mqtt_client.publish_sensor_data(ax, ay, az, gx, gy, gz)


            # Loop timing handling 
            
            # Calculate the remaining time to sleep to maintain the loop period, but allow a
            # Buffer to apply a hard lock on the loop period
            sleep_time = self.LOOP_TIME - (time.time() - loop_start_time) - .2*self.LOOP_TIME
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)  # Non-blocking sleep
                
            # Busy-wait until 10ms have passed since loop_start_time
            # ! This configuration is important to have steady loop periods 
            while True:
                loop_total_period = time.time() - loop_start_time
                if loop_total_period > RobotSystem.LOOP_TIME:
                    break
                
            loop_total_period = time.time() - loop_start_time

    def shutdown(self):
        # Shutdown the robot system 
        self.imu.close()
        self.mqtt_client.shutdown()        
        pass

def run_control_loop(robot):
    # Set the affinity of this thread to a specific core, e.g., core 1
    # p = psutil.Process()
    # p.cpu_affinity([1])
    asyncio.run(robot.control_loop())

def main():
    robot = RobotSystem()  # Initialize the robot system

    # Create a dedicated thread for running the control loop
    control_thread = threading.Thread(target=run_control_loop, args=(robot,))
    control_thread.start()

    try:
        # Keep the main thread running, or wait for the control thread to finish
        control_thread.join()
    except KeyboardInterrupt:
        robot.shutdown()  # Graceful exit for the robot
    finally:
        # No need to close the loop here since it's handled in the control thread
        pass

if __name__ == '__main__':
    main()