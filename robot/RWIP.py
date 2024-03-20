import time
from icm20948.lib.imu_lib import ICM20948
from fusion.AHRSfusion import AHRSfusion
import robot.LoopTimer as lt  
from motors.MN6007 import MN6007
import logging


# Create a logger
logger = logging.getLogger(__name__)

class RobotSystem:
    LOOP_TIME = 1/100  # 100 Hz control loop period

    def __init__(self, send_queue, receive_queue, start_motors=True):       
        self.send_queue = send_queue
        self.receive_queue = receive_queue

        # Init the IMU from library Config files are found in icm20948/configs dir and keep the imu settings
        imu1 = 'imu1.ini'
        self.imu = ICM20948(config_file=imu1)
        
        # Init sensor fusion
        self.sensor_fusion = AHRSfusion(self.imu._gyro_range, int(1/self.LOOP_TIME))
        
        #Initialize all actuators
        if start_motors:
            self.xmotor = MN6007()
        else:
            self.xmotor = None
        
        # Performance monitoring            
        self.gx_integral = 0
        
        self.itr = int(0)  # Cycle counter
        
    async def start(self):
        # Init actuators
        if self.xmotor is not None:
            await self.xmotor.start()
        await self.control_loop()

    async def control_loop(self):
        loop_period = .01
        while True:
            # Start Loop Timer and increment loop iteration
            loop_start_time = time.time()
            self.itr += 1
            
            # Poll the imu for positional data
            ax, ay, az, gx, gy, gz = self.imu.read_accelerometer_gyro(convert=True)
            
            #TODO Fuse sensor data or create a robot state estimate
            # Imported library from fusion/fusion.py which pulls from https://github.com/xioTechnologies/Fusion
            euler_angles, internal_states, flags = self.sensor_fusion.update((gx, gy, gz), (ax, ay, az), delta_time = loop_period)
            self.gx_integral = self.gx_integral + .01*gx #integrate the new reading to test gyro drift
                
            #TODO Update robot state and parameters
         
            #TODO Process a control decision using agent
            # Match a proportional response to the detected angle           
                        
            ## FIXED TIME EVENT (50-70% of way through loop period)
            #TODO Apply control decision to robot actuators
            # SET TORQUE
            setpoint = 0.2 * euler_angles[0] / 360
            if self.xmotor is not None:
                await self.xmotor.set_torque(torque=setpoint)
            
            #TODO Send all robot information to mqtt comms thread              
            if (self.itr % 20) == 0:
                self.send_imu_data(ax, ay, az, gx, gy, gz)
                self.send_euler_angles(euler_angles)
                if self.xmotor is not None:
                    self.send_motor_state(self.xmotor.state)
                  
            self.send_loop_time(loop_period*1e6)
            
            #TODO Check for new commands in receive queue 
            self.recieve_commands()  

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
            if remaining <= 0.0007:
                break
            time.sleep(remaining / 2)
            
        # Busy-while loop to get accurate cutoff at end
        while True:
            delay = time.time() - end_time
            if delay >= 0:
                return delay
            
    async def shutdown(self):
        # Shutdown the robot system 
        self.imu.close()     
        await self.xmotor.stop() 
            
    ############# IN/OUT Interface ################################ 
    #TODO Move all this communication interface to a different file 
            
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
            "euler_y": float(euler_angles[1]),  
            "euler_z": float(euler_angles[2])   
        }
        self.send_queue.put((topic, data))
        
    def send_motor_state(self, motor_state):
            topic = "robot/sensors/motor"
            data = motor_state        
            self.send_queue.put((topic, data))
    
    def send_loop_time(self, loop_time):
        topic = "robot/control/loop_time"
        data = {
            "loop_time": loop_time
        }
        self.send_queue.put((topic, data))      
        
    def recieve_commands(self):
        while not self.receive_queue.empty():
            message = self.receive_queue.get()
            #TODO Add a message handler to perform actions based on message
            
            logger.info(f'Command recieved: {message}')
            self.receive_queue.task_done()



