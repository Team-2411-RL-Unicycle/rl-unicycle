import os
import ctypes
import multiprocessing
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

def start_mqtt_process(telemetry_queue, command_queue):
    # Set a lower real-time priority for this process
    set_realtime_priority(priority=70)
    mqtt_client = MQTTClient(telemetry_queue, command_queue)
    mqtt_client.start()  # This should be a blocking call that runs the MQTT client

def main():
    setup_logging()
    logger = logging.getLogger()
    logger.debug("Logger initalized")
    
    # Escalate the process priority to max level
    set_realtime_priority()

    # Use multiprocessing.Manager to create queues that can be shared between processes
    manager = multiprocessing.Manager()
    telemetry_queue = manager.Queue()
    command_queue = manager.Queue()

    robot = RobotSystem(telemetry_queue, command_queue)
    # Start the MQTT communication in its own process
    mqtt_process = multiprocessing.Process(target=start_mqtt_process, 
                                            args=(telemetry_queue, 
                                            command_queue), 
                                            daemon=True)
    mqtt_process.start()

    # Start the control loop in the main process (or as a thread if required)
    try:
        robot.start()  # Assuming this is a blocking call; otherwise, handle it similarly to mqtt_client
    except KeyboardInterrupt:
        logger.info("Shutdown signal received")
    finally:
        # Perform necessary cleanup
        robot.shutdown()
        # Terminate the MQTT process
        mqtt_process.terminate()
        mqtt_process.join()
        logger.info("Cleanup complete. Exiting program.")

if __name__ == '__main__':
    main()