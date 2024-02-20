import sys, os, ctypes
import asyncio
import time

import threading
import psutil  

import LoopTimer as lt

# Define constants for the scheduling policy
SCHED_FIFO = 1  # FIFO real-time policy

class RobotSystem:
    LOOP_TIME = 1/100 # 100 Hz control loop period
    JITTER_OFFSET = 2e-6 # Loop time offset to smooth time average with jitter
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
        # variable to track control loop period
        loop_period = .01
        self.stopwatch_i2c = lt.LoopTimer(550)
        self.stopwatch_mqtt = lt.LoopTimer(550)
        while True:
            loop_start_time = time.time()

            self.itr += 1  # Increment the cycle counter 

            # //////////////// Sensor readings
            self.stopwatch_i2c.start()
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro(convert = True)
            self.stopwatch_i2c.stop()
            
            if self.itr % 500 == 0:
                print(f'Average sensor read {self.stopwatch_i2c.average_time()}')
                print(f' Max sensor read {max(self.stopwatch_i2c.get_execution_times())}')
            
            # //////////////// Control logic and motor commands based on sensor data           
            

            # MQTT communication for telemetry
              # Convert loop period measurement to microseconds            
            self.mqtt_client.publish_loop_time(loop_period* 1e6 )            
            if (self.itr % 20) == 0:  # Downsample 
                self.mqtt_client.publish_sensor_data(ax, ay, az, gx, gy, gz)
                
            # Control loop end timer
            end_time = loop_start_time + RobotSystem.LOOP_TIME - RobotSystem.JITTER_OFFSET
            loop_delay  = self.precise_delay_until(end_time)
            loop_period = time.time()- loop_start_time
            
    def precise_delay_until(self, end_time):
        # Sleep for the bulk of the time, leaving a small amount (e.g., 0.9ms) for busy-waiting
        while True:
            now = time.time()
            remaining = end_time - now
            if remaining <= 0.0005:  # Adjust this value based on your system's timing resolution and accuracy
                break
            time.sleep(remaining / 2)  # Sleep for half the remaining time to reduce the chance of oversleeping

        # Busy-wait for fine-tuned precision
        while True:
            delay = time.time() - end_time
            if delay>= 0:
                return delay
                       

    def shutdown(self):
        # Shutdown the robot system 
        self.imu.close()
        self.mqtt_client.shutdown()        
        pass

class SchedParam(ctypes.Structure):
    _fields_ = [('sched_priority', ctypes.c_int)]
    
# Function to set real-time priority
def set_realtime_priority(priority=99):
    libc = ctypes.CDLL('libc.so.6')
    param = SchedParam(priority)
    # Set the scheduling policy to FIFO and priority
    if libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param)) != 0:
        raise ValueError("Failed to set real-time priority. Check permissions.")

def run_control_loop(robot):
    try:
        set_realtime_priority()  # Set real-time priority
    except ValueError as e:
        print(e)
        return  # Exit if setting real-time priority fails

    p = psutil.Process(os.getpid())
    p.cpu_affinity([1])  # Set the process to run on CPU core 1

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