import sys, os
import asyncio
import time



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
        loop_total_period = 0
        while True:
            self.itr += 1  # Increment the cycle counter 
            loop_start_time = time.time()            
            
            # Sensor readings
            ax,ay,az,gx,gy,gz= self.imu.read_accelerometer_gyro(convert = True)
            
            # Control logic and motor commands based on sensor data                    
            # MQTT communication for telemetry
            loop_delay = (loop_total_period - RobotSystem.LOOP_TIME) * 1e6  # Convert to micro seconds
            self.mqtt_client.publish_loop_time(loop_delay)
            
            if (self.itr % 20) == 0:  # Downsample 
                self.mqtt_client.publish_sensor_data(ax,ay,az,gx,gy,gz)
                
            # Busy-wait until 10ms have passed since loop_start_time
            # ! This configuration is important to have steady loop periods 
            while True:
                loop_total_period = time.time() - loop_start_time
                if loop_total_period > RobotSystem.LOOP_TIME:
                    break

    def shutdown(self):
        # Shutdown the robot system 
        self.imu.close()
        self.mqtt_client.shutdown()        
        pass

def main():
    robot = RobotSystem()  # Initialize the robot system
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(robot.control_loop())
    except KeyboardInterrupt:
        robot.shutdown()  # Graceful exit for the robot
    finally:
        loop.close()


if __name__ == '__main__':
    main()