import os
import ctypes
import queue
import threading
from robot.robot_system import RobotSystem
from communication.mqtt import MQTTClient
import logging.config

# Define constants for the scheduling policy
SCHED_FIFO = 1  # FIFO real-time policy

class SchedParam(ctypes.Structure):
    _fields_ = [('sched_priority', ctypes.c_int)]
    
def set_realtime_priority(priority=99):
    libc = ctypes.CDLL('libc.so.6')
    param = SchedParam(priority)
    # Set the scheduling policy to FIFO and priority for the entire process (0 refers to the current process)
    if libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param)) != 0:
        raise ValueError("Failed to set real-time priority. Check permissions.")
    
def setup_logging():
    # Get the directory of the current script
    dir_path = os.path.dirname(os.path.realpath(__file__))
    # Construct the path to logging.ini relative to the current script
    logging_ini_path = os.path.join(dir_path, 'log/logging.ini')
    # Use the logging_ini_path to load the configuration (edit this file to configure logger settings)
    logging.config.fileConfig(logging_ini_path, disable_existing_loggers=False)


def main():
    setup_logging()
    logger = logging.getLogger()
    logger.debug("Logger initalized")
    
    # Escalate the process priority to max level
    set_realtime_priority()

    # Create thread-safe queues for MQTT communication
    telemetry_queue = queue.Queue()
    command_queue = queue.Queue()

    # Initialize MQTT client and robot system with queues
    # The Queues act as inter-thread communication buffers
    mqtt_client = MQTTClient(telemetry_queue, command_queue)
    robot = RobotSystem(telemetry_queue, command_queue)

    # Start the control loop and MQTT communication in their own daemon threads
    # This calls the start method in each of them serving as thread entry point
    # See the start method in each class for implementation
    control_thread = threading.Thread(target=robot.start, daemon=True)
    mqtt_thread = threading.Thread(target=mqtt_client.start, daemon=True)
    control_thread.start()
    mqtt_thread.start()

    try:
        # Use thread.join() with a timeout to keep the main thread responsive
        while True:
            control_thread.join(timeout=1)
            mqtt_thread.join(timeout=1)
    except KeyboardInterrupt:
        logger.info("Shutdown signal received")
    finally:
        # Perform necessary cleanup
        robot.shutdown()
        mqtt_client.shutdown()
        logger.info("Cleanup complete. Exiting program.")

if __name__ == '__main__':
    main()