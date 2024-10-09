import asyncio
import multiprocessing
import signal
import time

from rluni.communication.mqtt import MQTTClient
from rluni.robot import teledata as td


def start_mqtt_process(telemetry_queue, command_queue):
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

        mqtt_client = MQTTClient(telemetry_queue, command_queue)
        mqtt_client.start()  # This should be a blocking call that runs the MQTT client
    except Exception as e:
        raise


def main():
    telemetry_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()

    # Start the MQTT communication in its own process
    mqtt_process = multiprocessing.Process(
        target=start_mqtt_process,
        args=(telemetry_queue, command_queue),
    )

    mqtt_process.start()  # Start the process!

    # Check the time cost on creating and destroying the dataclass objects
    n_iters = 5

    for _ in range(n_iters):
        start = time.time()
        imu = td.IMUData(1.2, 1.2, 1.4, 1.1, 1.2, 1.3)
        euler = td.EulerAngles(1.2, 1.2, 1.4)
        motor = td.MotorState(1.2, 1.2, 1.4)
        motor_electrical = td.MotorElectrical(1.2, 1.2, 1.4, 1.1, 1.2, 1)
        control = td.ControlData(1.2)
        debug = td.DebugData(1.2)

        # Optionally send either a batch or a single item
        if _ % 2 == 0:
            # Send a batch of telemetry data
            list_of_data = [imu, euler, motor, motor_electrical, control, debug]
            telemetry_queue.put(list_of_data)
        else:
            # Send a single telemetry object
            telemetry_queue.put(imu)

        end = time.time()
        print(f"Time taken to create and send dataclass objects: {end-start}")

    # Ensure that the MQTT process is properly terminated
    while not telemetry_queue.empty():
        time.sleep(0.1)

    mqtt_process.terminate()
    mqtt_process.join()


if __name__ == "__main__":
    main()
