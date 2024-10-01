import argparse
import asyncio
import ctypes
import logging.config
import multiprocessing
import os

from rluni.communication.mqtt import MQTTClient
from rluni.robot.RWIP import RobotSystem


class SchedParam(ctypes.Structure):
    """Python wrapper for the sched_param struct in C"""

    _fields_ = [("sched_priority", ctypes.c_int)]


def set_realtime_priority(priority=99):
    """
    Set the process's real-time priority.

    Parameters:
        priority (int): The real-time priority level to set (1-99).
    """
    SCHED_FIFO = 1
    libc = ctypes.CDLL("libc.so.6")
    param = SchedParam(priority)
    result = libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param))
    if result != 0:
        errno = ctypes.get_errno()
        if errno == 1:  # Operation not permitted
            raise PermissionError(
                "Failed to set real-time priority. Check permissions."
            )
        else:
            raise ValueError(f"Failed to set real-time priority. Error code: {errno}")


def setup_logging(config_path=None):
    """
    Set up logging configuration.

    Parameters:
        config_path (str, optional): Path to the logging configuration file.
                                     Defaults to 'log/logging.ini' relative to this script.
    """
    if config_path is None:
        # Get the directory of the current script
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # Default logging configuration file
        config_path = os.path.join(dir_path, "log", "logging.ini")
    try:
        logging.config.fileConfig(config_path, disable_existing_loggers=False)
    except Exception as e:
        print(f"Failed to load logging configuration from {config_path}: {e}")
        logging.basicConfig(level=logging.INFO)


def start_mqtt_process(telemetry_queue, command_queue):
    """
    Start the MQTT client process with a lower real-time priority.

    Parameters:
        telemetry_queue (multiprocessing.Queue): Queue for sending telemetry data.
        command_queue (multiprocessing.Queue): Queue for receiving commands.
    """
    try:
        # Set a lower real-time priority for this process
        set_realtime_priority(priority=70)
        mqtt_client = MQTTClient(telemetry_queue, command_queue)
        mqtt_client.start()  # This should be a blocking call that runs the MQTT client
    except Exception as e:
        logging.exception(f"MQTT process encountered an error: {e}")
        raise


def parse_args() -> argparse.Namespace:
    """
    Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Start the robot system with optional motor control."
    )
    parser.add_argument(
        "-nm",
        "--no-motors",
        action="store_true",
        help="Do not start the motors of the robot system.",
    )
    parser.add_argument(
        "-c",
        "--controller",
        type=str,
        default="test",
        choices=["pid", "rl", "test"],
        help="Select the type of controller: pid, rl, or test.",
    )
    parser.add_argument(
        "-cfg",
        "--config-file",
        type=str,
        help="Specify an alternative RWIP configuration YAML file.",
    )
    return parser.parse_args()


def cancel_pending_tasks(loop):
    """
    Cancel all pending asyncio tasks to clear the event loop completely.

    Parameters:
        loop (asyncio.AbstractEventLoop): The event loop to operate on.
    """
    for task in asyncio.all_tasks(loop):
        task.cancel()
        try:
            loop.run_until_complete(task)
        except asyncio.CancelledError:
            pass  # Task cancellation is expected


def main():
    # Get command line flags
    args = parse_args()

    # Setup the logging configuration
    setup_logging()
    logger = logging.getLogger()
    logger.debug("Logger initalized")

    try:
        # Escalate the process priority to max level
        set_realtime_priority(priority=99)
    except PermissionError as e:
        logger.error(f"Failed to set real-time priority: {e}")
        return

    # Use multiprocessing.Manager to create queues that can be shared between processes
    with multiprocessing.Manager() as manager:
        telemetry_queue = manager.Queue()
        command_queue = manager.Queue()

        # Initialize the robot with the command-line argument
        robot = RobotSystem(
            telemetry_queue,
            command_queue,
            start_motors=not args.no_motors,
            controller_type=args.controller,
            config_file=args.config_file,
        )
        # Start the MQTT communication in its own process
        mqtt_process = multiprocessing.Process(
            target=start_mqtt_process,
            args=(telemetry_queue, command_queue),
            daemon=True,
        )
        mqtt_process.start()

        # Start the control loop in the main process
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            # Schedule and run the RobotSystem.start coroutine
            loop.run_until_complete(robot.start())
        except KeyboardInterrupt:
            logger.info("Shutdown signal received")
        except Exception as e:
            logger.exception(f"An unexpected error occurred: {e}")
        finally:
            # Cancel any pending asyncio tasks
            cancel_pending_tasks(loop)

            # Shutdown the robot system
            loop.run_until_complete(robot.shgitown())

            # Terminate and clean up the MQTT process
            mqtt_process.terminate()
            mqtt_process.join(timeout=5)
            if mqtt_process.is_alive():
                mqtt_process.kill()
                logger.warning(
                    "MQTT process did not terminate gracefully, forcing termination."
                )

            logger.info("Cleanup complete. Exiting program.")

            # Close the asyncio event loop
            loop.close()


if __name__ == "__main__":
    main()
