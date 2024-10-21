from rluni.motors.MN6007 import MN6007
import asyncio
import math
import time


async def main():
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    motor1 = MN6007(1)
    motor4 = MN6007(4)

    # In case the controller had faulted previously, at the start of
    # this script we send the stop command in order to clear it.
    await motor1.stop()
    await motor4.stop()
    request1_times = []
    request2_times = []
    total_request_times = []

    start_time = time.time()
    try:
        while True:
            elapsed_time = time.time() - start_time
            torque = 0.07 * math.sin(2*elapsed_time)
            request_start_time = time.time()
            feedback_1 = await motor1.set_torque(torque)
            request1_end_time = time.time()
            # await asyncio.sleep(0.03)  # Small delay to prevent overwhelming the motors
            feedback_2 = await motor4.set_torque(torque)
            request2_end_time = time.time()
            
            request1_times.append(request1_end_time - request_start_time)
            request2_times.append(request2_end_time - request1_end_time)
            total_request_times.append(request2_end_time - request_start_time)

            await asyncio.sleep(0.03)  # Small delay to prevent overwhelming the motors
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping motors...")
    finally:
        await motor1.stop()
        await motor4.stop()
        with open("final_request_times.csv", "w") as final_file:
            final_file.write("request1_times,request2_times,total_request_times\n")
            for r1, r2, tr in zip(request1_times, request2_times, total_request_times):
                final_file.write(f"{r1},{r2},{tr}\n")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exited... Double check that the motors are off and not ringing!")





