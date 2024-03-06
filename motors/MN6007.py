import time
import moteus
import logging

# Create a logger
logger = logging.getLogger(__name__)

class MN6007:
    MAX_ALLOWABLE_ACCEL = 3.0
    MAX_ALLOWABLE_VEL = 8.0
    def __init__(self):
        self.controller = moteus.Controller(transport=None)

    def start(self):
        # Clear faults
        self.controller.set_stop()
        
    def set_position(self, position, velocity, accel_limit = 3.0, velocity_limit = 8.0, query = True):
        '''
        Send a position or velocity command to the motor
        '''
        if accel_limit > self.MAX_ALLOWABLE_ACCEL:
            accel_limit = self.MAX_ALLOWABLE_ACCEL
            logger.warning(f'Acceleration limit set too high. Setting to {self.MAX_ALLOWABLE_ACCEL}')
        if velocity_limit > self.MAX_ALLOWABLE_VEL:
            velocity_limit = self.MAX_ALLOWABLE_VEL
            logger.warning(f'Velocity limit set too high. Setting to {self.MAX_ALLOWABLE_VEL}')
    
        feedback = self.controller.set_position(
            position=position,
            velocity=velocity,
            accel_limit=accel_limit,
            velocity_limit=velocity_limit,
            query=query,
        )
        
        return feedback

    def stop(self):
        # Ensure controller is stopped and resources are cleaned up properly
        if self.controller:
            self.controller.set_stop()

# Usage
if __name__ == '__main__':
    actuator = MN6007()
    actuator.start()
