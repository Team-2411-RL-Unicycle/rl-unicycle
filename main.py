import argparse
import asyncio
import ctypes
import logging.config
import multiprocessing
import os
import signal
import time

from rluni.communication.mqtt import MQTTClient
from rluni.robot.robot import RobotSystem


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
        save_path (str): Path to save the log file.
    """
    if config_path is None:
        # Get the directory of the current script
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # Default logging configuration file
        config_path = os.path.join(dir_path, "log", "logging.ini")

        # Create logfile and make sure dir exists
        save_dir = os.path.join(dir_path, "log", "saved_logs")
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        unique_id = str(int(time.time()))
        save_path = os.path.join(save_dir, f"robot_log_{unique_id}.log")
    try:
        # Pass the save_path via the 'defaults' parameter
        logging.config.fileConfig(
            config_path,
            defaults={"logfilename": save_path},
            disable_existing_loggers=False,
        )
    except Exception as e:
        print(f"Failed to load logging configuration from {config_path}: {e}")
        logging.basicConfig(level=logging.INFO)


def start_mqtt_process(telemetry_queue, command_queue, shutdown_event):
    """
    Start the MQTT client process with a lower real-time priority.

    Parameters:
        telemetry_queue (multiprocessing.Queue): Queue for sending telemetry data.
        command_queue (multiprocessing.Queue): Queue for receiving commands.
    """
    signal.signal(
        signal.SIGINT, signal.SIG_IGN
    )  # Ignore hard kill signals for rapid termination
    try:
        # Set a lower real-time priority for this process
        set_realtime_priority(priority=70)
        mqtt_client = MQTTClient(telemetry_queue, command_queue, shutdown_event)
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
    # TODO: this needs to be removed or changed to actually work
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
        choices=["pid", "rl", "mpc", "test", "lqr"],
        help="Select the type of controller: pid, lqr, or rl.",
    )
    parser.add_argument(
        "-cfg",
        "--config-file",
        type=str,
        help="Specify an alternative robot configuration YAML file.",
    )
    parser.add_argument(
        "-mcfg", "--motor-config", type=str, help="Specify which motors should be on."
    )
    return parser.parse_args()


async def main():
    args = parse_args()
    setup_logging()
    logger = logging.getLogger()
    logger.debug("Logger initialized")

    try:
        set_realtime_priority(priority=99)
    except PermissionError as e:
        logger.error(f"Failed to set real-time priority: {e}")
        return

    telemetry_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()
    shutdown_event = multiprocessing.Event()

    # Start the MQTT process
    mqtt_process = multiprocessing.Process(
        target=start_mqtt_process,
        args=(telemetry_queue, command_queue, shutdown_event),
    )
    mqtt_process.start()

    # Initialize and start the robot
    robot = RobotSystem(
        telemetry_queue,
        command_queue,
        run_motors=not args.no_motors,
        controller_type=args.controller,
        config_file=args.config_file,
        motor_config=args.motor_config,
    )

    try:
        await robot.start(shutdown_event)
    except KeyboardInterrupt:
        logger.info("Shutdown signal received")
        shutdown_event.set()
    except Exception as e:
        logger.exception(f"An unexpected error occurred: {e}")
    finally:
        await robot.shutdown()

        # Terminate and clean up the MQTT process
        shutdown_event.set()
        mqtt_process.join(timeout=5)
        if mqtt_process.is_alive():
            mqtt_process.kill()
            logger.warning(
                "MQTT process did not terminate gracefully, forcing termination."
            )

        logger.info("Cleanup complete. Exiting program.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exited... Double check that the motors are off and not ringing!")
