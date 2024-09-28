import argparse
import asyncio
import ctypes
import logging.config
import multiprocessing
import os

from rluni.communication.mqtt import MQTTClient
from rluni.robot.RWIP import RobotSystem

# Define constants for the scheduling policy
SCHED_FIFO = 1  # FIFO real-time policy


class SchedParam(ctypes.Structure):
    _fields_ = [("sched_priority", ctypes.c_int)]


def set_realtime_priority(priority=99):
    libc = ctypes.CDLL("libc.so.6")
    param = SchedParam(priority)
    # Set the scheduling policy to FIFO and priority for the entire process (0 refers to the current process)
    if libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param)) != 0:
        raise ValueError("Failed to set real-time priority. Check permissions.")


def setup_logging():
    # Get the directory of the current script
    dir_path = os.path.dirname(os.path.realpath(__file__))
    # Construct the path to logging.ini relative to the current script
    logging_ini_path = os.path.join(dir_path, "log/logging.ini")
    # Use the logging_ini_path to load the configuration (edit this file to configure logger settings)
    logging.config.fileConfig(logging_ini_path, disable_existing_loggers=False)


def start_mqtt_process(telemetry_queue, command_queue):
    # Set a lower real-time priority for this process
    set_realtime_priority(priority=70)
    mqtt_client = MQTTClient(telemetry_queue, command_queue)
    mqtt_client.start()  # This should be a blocking call that runs the MQTT client


def parse_args():
    # Setup command-line argument parsing
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
        choices=["pid", "rl", "test", "lqr"],
        help="Select the type of controller: pid, lqr, or rl.",
    )
    args = parser.parse_args()
    return args


def cancel_pending_tasks(loop):
    try:
        current_task = asyncio.current_task(loop)
    except RuntimeError:
        current_task = None  # No current task if no running loop

    for task in asyncio.all_tasks(loop):
        if task is not current_task:
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

    # Escalate the process priority to max level
    set_realtime_priority()

    # Use multiprocessing.Manager to create queues that can be shared between processes
    manager = multiprocessing.Manager()
    telemetry_queue = manager.Queue()
    command_queue = manager.Queue()

    # Initialize the robot with the command-line argument
    robot = RobotSystem(
        telemetry_queue,
        command_queue,
        start_motors=not args.no_motors,
        controller_type=args.controller,
    )
    # Start the MQTT communication in its own process
    mqtt_process = multiprocessing.Process(
        target=start_mqtt_process, args=(telemetry_queue, command_queue), daemon=True
    )
    mqtt_process.start()

    # Start the control loop in the main process (or as a thread if required)
    loop = asyncio.get_event_loop()
    try:
        # Schedule and run the RobotSystem.start coroutine
        loop.run_until_complete(robot.start())
    except KeyboardInterrupt:
        logger.info("Shutdown signal received")
    finally:
        # Cancel any pending asyncio tasks
        cancel_pending_tasks(loop)

        # Shutdown the robot system
        loop.run_until_complete(robot.shutdown())

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
