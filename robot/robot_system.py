import time
from icm20948.lib.imu_lib import ICM20948
from fusion.fusion import AHRSfusion
import robot.LoopTimer as lt  
import logging

# Create a logger
logger = logging.getLogger(__name__)

class RobotSystem:
    LOOP_TIME = 1/100  # 100 Hz control loop period

    def __init__(self, send_queue, receive_queue):       
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        # Init the IMU from library Config files are found in icm20948/configs dir and keep the imu settings
        imu1 = 'imu1.ini'
        self.imu = ICM20948(config_file=imu1)
        
        # Init sensor fusion
        self.sensor_fusion = AHRSfusion(self.imu._gyro_range, int(1/self.LOOP_TIME))

        #TODO Initialize all actuators
        self.xmotor = None
        self.ymotor = None
        self.zmotor = None

        self.itr = int(0)  # Cycle counter
        self.stopwatch_i2c = lt.LoopTimer(550)
        
        self.gx_test = 0
        
    def start(self):
        self.control_loop()

    def control_loop(self):
        loop_period = .01
        while True:
            loop_start_time = time.time()
            self.itr += 1

            self.stopwatch_i2c.start()
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro(convert=True)
            self.stopwatch_i2c.stop()    
            
            if self.itr % 500 == 0:
                logger.debug(f'Average sensor read {self.stopwatch_i2c.average_time()}')
                logger.debug(f' Max sensor read {max(self.stopwatch_i2c.get_execution_times())}')

            #TODO Fuse sensor data or create a robot state estimate
            # Imported library from fusion/fusion.py which pulls from https://github.com/xioTechnologies/Fusion
            euler_angles, internal_states, flags = self.sensor_fusion.update((gx, gy, gz), (ax, ay, az), delta_time = loop_period)
            self.gx_test = self.gx_test + .01*gx #integrate the new reading to test gyro drift
            
            #TODO Check for new commands in receive queue 
                # remote changes to robot parameters)     
                
            #TODO Update robot state and parameters
            
            #TODO Process a control decision using agent
            
            ## FIXED TIME EVENT (50-70% of way through loop period)
            #TODO Apply control decision to robot actuators
            
            #TODO Send all robot information to mqtt comms thread              
            if (self.itr % 20) == 0:
                self.send_imu_data(ax, ay, az, gx, gy, gz)
                self.send_euler_angles(euler_angles)

                logger.debug(f'gyro integral = {self.gx_test}')
                logger.debug(f'accel_error = {internal_states[0]}')
                  
            self.send_loop_time(loop_period*1e6)

            end_time = loop_start_time + RobotSystem.LOOP_TIME
            loop_delay = self.precise_delay_until(end_time)
            loop_period = time.time() - loop_start_time

    def precise_delay_until(self, end_time):
        """
        Sleep until the specified end time, then return the delay time (overshoot)
        """
        
        # Lower accuracy sleep loop
        while True:
            now = time.time()
            remaining = end_time - now
            if remaining <= 0.0005:
                break
            time.sleep(remaining / 2)
            
        # Busy-while loop to get accurate cutoff at end
        while True:
            delay = time.time() - end_time
            if delay >= 0:
                return delay
            
            
    def send_imu_data(self, ax, ay, az, gx, gy, gz):
        topic = "robot/sensors/imu"
        data = {
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x" : gx,
            "gyro_y" : gy,
            "gyro_z" : gz,
        }
        self.send_queue.put((topic, data))
        
    def send_euler_angles(self, euler_angles):
        topic = "robot/state/angles"
        data = {
            "euler_x": float(euler_angles[0]),  # Convert to Python float
            "euler_y": float(euler_angles[1]),  # Convert to Python float
            "euler_z": float(euler_angles[2])   # Convert to Python float
        }
        self.send_queue.put((topic, data))
    
    def send_loop_time(self, loop_time):
        topic = "robot/control/loop_time"
        data = {
            "loop_time": loop_time
        }
        self.send_queue.put((topic, data))      

    def shutdown(self):
        # Shutdown the robot system 
        self.imu.close()      
        pass
