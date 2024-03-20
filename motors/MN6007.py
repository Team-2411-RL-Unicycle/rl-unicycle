import time
import moteus
from moteus.moteus import Register 
import math
import asyncio
import logging


# Create a logger
logger = logging.getLogger(__name__)


class MN6007:
    MAX_ALLOWABLE_ACCEL = 200.0
    MAX_ALLOWABLE_VEL = 20.0
    MAX_ALLOWABLE_TORQUE = 0.1 # Set low for now
    TIMEOUT_SECONDS = .1 #Seconds
        
    def __init__(self):
        # Transport = None searches out for the first available CANFD Device
        self._c = moteus.Controller(transport=None)  
        logger.debug(f'Connected to Motor Controller: {self._c}')      
        # Motor system state (Dictionary of register values)
        self.state = None        

    async def start(self):
        """
        Attempts to clear any existing faults and initialize the motor. If the motor
        fails to respond within a predefined timeout, it logs an error and raises
        a RuntimeError to indicate the failure to start the motor, triggering a
        shutdown sequence for safety.

        Raises:
            RuntimeError: If the motor cannot be initialized within the timeout period.
        """
        try:
            await asyncio.wait_for(self._c.set_stop(), self.TIMEOUT_SECONDS)
            logger.info("Motor initialized successfully")
        except asyncio.TimeoutError:
            logger.error("Failed to initialize motor: Timeout while connecting to the moteus driver. Please check the connection.")
            # Initiate robot shutdown sequence here or raise an exception to be handled by the caller
            self.stop()
            raise RuntimeError("Failed to initialize motor: Operation timed out.")

        
    async def update_state(self):
        # Query the motor state
        # Throw exception if stuck in await function
        state = await self._c.set_position(position=math.nan, query=True)
        # Parse the state and update the motor object's state
        self.parse_state(state)
                
    def parse_state(self, result):
        # Convert the Result object's values to a dictionary and update the state
        self.state = {Register(key).name: value for key, value in result.values.items()}
            
    def formatted_state(self):
        formatted_state = "MODE: {MODE: >4}, POSITION: {POSITION: >7.4f}, VELOCITY: {VELOCITY: >7.4f}, " \
                          "TORQUE: {TORQUE: >7.4f}, VOLTAGE: {VOLTAGE: >5.1f}, TEMPERATURE: {TEMPERATURE: >5.1f}, " \
                          "FAULT: {FAULT: >4}".format(**self.state)

        return formatted_state
    
    async def set_position(self, position, velocity, accel_limit = 3.0, velocity_limit = 8.0, query = True):
        '''
        Send a position or velocity command to the motor
        '''
        if accel_limit > self.MAX_ALLOWABLE_ACCEL:
            accel_limit = self.MAX_ALLOWABLE_ACCEL
            logger.warning(f'Acceleration limit set too high. Setting to {self.MAX_ALLOWABLE_ACCEL}')
        if velocity_limit > self.MAX_ALLOWABLE_VEL:
            velocity_limit = self.MAX_ALLOWABLE_VEL
            logger.warning(f'Velocity limit set too high. Setting to {self.MAX_ALLOWABLE_VEL}')
    
        feedback = await self._c.set_position(
            position=position,
            velocity=velocity,
            accel_limit=accel_limit,
            velocity_limit=velocity_limit,
            query=query,
        )
        
        self.parse_state(feedback)
        
        return feedback
    
    async def set_torque(self, torque = 0.0, min_torque = -0.3, max_torque = 0.3, query = True):
        '''
        Send a torque command to the motor
        '''
        if abs(torque) > self.MAX_ALLOWABLE_TORQUE:
            logger.warning(f'Torque set outside bounds. Attempted to set to {torque:.2f}N*m. Setting to {self.MAX_ALLOWABLE_TORQUE * (1 if torque > 0.0 else -1)}')
            torque = self.MAX_ALLOWABLE_TORQUE * (1 if torque > 0.0 else -1)
        feedback = await self._c.set_position(
            position=math.nan,
            kp_scale=0.0,
            kd_scale=0.0,
            feedforward_torque=torque,
            maximum_torque=self.MAX_ALLOWABLE_TORQUE,     
            query=query,
        )
        
        self.parse_state(feedback)
        
        return feedback

    async def stop(self):
        # Ensure controller is stopped and resources are cleaned up properly
        if self._c:
            await self._c.set_stop()

##################################################################
# Testing functions

async def test_loop(actuator):
    ''' 
    Testing loop for motor
    '''
    while True:
        
        await actuator.update_state()
        await asyncio.sleep(.1)
        print(actuator.formatted_state())   
        results = await actuator.set_position(
                position=3,
                velocity=0
        )


async def main(actuator):
    await actuator.start()   

    iter = 0.0
    while True:
        iter = iter + 1
        setpoint = iter * .01
        results = await actuator.set_position(
            position=setpoint,
            velocity=0
        )
        results = actuator.parse_state(results)
        print(actuator.formatted_state())
        await asyncio.sleep(0.01)

# Test usage
if __name__ == '__main__':
    # Configure logging level to DEBUG to show all messages
    logging.basicConfig(level=logging.DEBUG)  # This line configures the root logger level
    actuator = MN6007()
    loop = asyncio.get_event_loop()


    
    try:
        loop.run_until_complete(test_loop(actuator))
    except KeyboardInterrupt:
        print("Exiting...")
        # Since loop.run_until_complete is no longer running here, we cannot use 'await' directly
        # We need to create a new task to run the stop coroutine
        loop.run_until_complete(actuator.stop())
    finally:
        loop.close()
