import time
import json
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
    MAX_ALLOWABLE_TORQUE = 1.4 # Set low for now
    TIMEOUT_SECONDS = .1 #Seconds
        
    def __init__(self):

        qr = moteus.QueryResolution()
        qr.q_current = moteus.INT32
        qr.d_current = moteus.INT32
        qr.voltage = moteus.INT32
        # Transport = None searches out for the first available CANFD Device
        self._c = moteus.Controller(transport=None, query_resolution=qr)  
        logger.debug(f'Connected to Motor Controller: {self._c}')      
        # Motor system state (Dictionary of register values)
        self.state = None   
        self.isfaulted = False
        self.fault_codes = self.load_fault_codes()
    
    def load_fault_codes(self):
        # Load fault codes from a JSON file
        with open('motors/fault_codes.json', 'r') as f:
            return json.load(f)

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
        except asyncio.TimeoutError:
            logger.error("Failed to initialize motor: Timeout while connecting to the moteus driver. Please check the connection.")
            # Initiate robot shutdown sequence here or raise an exception to be handled by the caller
            self.stop()
            raise RuntimeError("Failed to initialize motor: Operation timed out.")
        
        await self.update_state()

        
    async def update_state(self):
        """
        Queries and updates the motor state. If the operation does not complete
        within the specified timeout, logs an error and raises a TimeoutError.

        Raises:
            asyncio.TimeoutError: If the operation times out.
        """
        try:
            state = await asyncio.wait_for(
                # moteus query command (can also make custom queries)
                self._c.query(),
                self.TIMEOUT_SECONDS
            )
            self.parse_state(state)           
            self.check_fault()
            return state
        except asyncio.TimeoutError:
            logger.error("Failed to update motor state: Operation timed out.")
            raise  # Re-raise the exception for the caller to handle
                
    def parse_state(self, result):
        # Convert the Result object's values to a dictionary and update the state
        self.state = {Register(key).name: value for key, value in result.values.items()}
            
    def formatted_state(self):
        formatted_state = "MODE: {MODE: >4}, POSITION: {POSITION: >7.4f}, VELOCITY: {VELOCITY: >7.4f}, " \
                          "TORQUE: {TORQUE: >7.4f}, VOLTAGE: {VOLTAGE: >5.1f}, TEMPERATURE: {TEMPERATURE: >5.1f}, " \
                          "FAULT: {FAULT: >4}".format(**self.state)

        return formatted_state
    
    def check_fault(self):
        # Check if the motor is faulted
        if self.state['FAULT'] != 0 and not self.isfaulted:
            # Get the fault code from the dictionary
            fault_code = self.fault_codes.get(str(self.state['FAULT']), "Unknown fault")
            logger.error(f"Motor fault detected: {fault_code}")
            self.isfaulted = True
    
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
        
        # self.parse_state(feedback)
        return feedback
    
    async def set_torque(self, torque=0.0, min_torque=-0.3, max_torque=0.3, query=False):
        """
        Sends a torque command to the motor. If the operation does not complete within the
        specified timeout, logs an error and raises a TimeoutError.

        Args:
            torque: Desired torque for the motor.
            min_torque: Minimum allowable torque.
            max_torque: Maximum allowable torque.
            query: Whether to query the motor state after setting the torque.

        Raises:
            asyncio.TimeoutError: If the operation times out.
        """
        if abs(torque) > self.MAX_ALLOWABLE_TORQUE:
            error_msg = f'Torque set outside bounds. Attempted to set to {torque:.2f}N*m. Bounds are +/- {self.MAX_ALLOWABLE_TORQUE}'
            logger.error(error_msg)
            # Something went wrong, turn off the motor and robot
            self.stop()
            raise ValueError(error_msg)

        if abs(torque) > max_torque:
            torque = max_torque * (1 if torque > 0.0 else -1) 
            warning_msg = f'Torque set outside bounds. Attempted to set to {torque:.2f}N*m. Bounds are +/- {max_torque}' 
            logger.warning(warning_msg)


        try:
            feedback = await asyncio.wait_for(
                self._c.set_position(
                    position=math.nan,
                    kp_scale=0.0,
                    kd_scale=0.0,
                    feedforward_torque=torque,
                    maximum_torque=self.MAX_ALLOWABLE_TORQUE,
                    query=query,
                ),
                self.TIMEOUT_SECONDS
            )
            # self.parse_state(feedback)
            return feedback
        
        except asyncio.TimeoutError:
            logger.error("Failed to set motor torque: Operation timed out.")
            raise  # Re-raise the exception for the caller to handle

    async def stop(self):
            # Ensure controller is stopped and resources are cleaned up properly
            if self._c:
                attempts = 0
                max_attempts = 3  # Try stopping multiple times if failed to stop

                while attempts < max_attempts:
                    try:
                        await asyncio.wait_for(self._c.set_stop(), self.TIMEOUT_SECONDS)
                        logger.debug("Motor successfully stopped")
                        break  # Exit the loop if the stop operation was successful
                    except asyncio.TimeoutError:
                        attempts += 1  # Increment the attempt counter
                        logger.error(f"Motor stop operation timed out, attempt {attempts} of {max_attempts}")

                        if attempts == max_attempts:
                            logger.critical("Failed to stop the motor after multiple attempts")
                            raise

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
