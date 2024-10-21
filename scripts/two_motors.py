from rluni.motors.MN6007 import MN6007
from rluni.icm20948.imu_lib import ICM20948
from rluni.fusion.AHRSfusion import AHRSfusion
from rluni.robot import teledata as td
import importlib.resources as pkg_resources
import asyncio
import math
import time
import moteus
import numpy as np


async def main():
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    # motor1 = MN6007(1)
    # motor4 = MN6007(4)
    with pkg_resources.path('rluni.configs.imu', 'imu1.yaml') as config_file_path:
        imu_config_path = str(config_file_path)

    imu = ICM20948(config_file=imu_config_path)
    sensor_fusion = AHRSfusion(sample_rate=100, config_file=imu_config_path)
    fdcanusb_transport = moteus.Fdcanusb('/dev/serial/by-id/usb-mjbots_fdcanusb_B1457958-if00')
    motors = {
        1: moteus.Controller(1, transport=fdcanusb_transport),
        4: moteus.Controller(4, transport=fdcanusb_transport)
    }

    # In case the controller had faulted previously, at the start of
    # this script we send the stop command in order to clear it.
    # await motor1.stop()
    # await motor4.stop()
    await fdcanusb_transport.cycle([motor.make_stop() for motor in motors.values()])

    # request1_times = []
    # request2_times = []
    # total_request_times = []
    request_times = []
    query_times = []
    imu_times = []
    fusion_times = []
    tasks = []


    start_time = time.time()
    try:
        while True:
            loop_start_time = time.time()
            elapsed_time = time.time() - start_time
            torque = 0.07 * math.sin(2*elapsed_time)
            # commands = [
            #     motors[1].make_position(
            #         position=math.nan,
            #         kp_scale=0.0,
            #         kd_scale=0.0,
            #         feedforward_torque=torque,
            #         maximum_torque=0.1,
            #         query=True,
            #     ),
            #     motors[4].make_position(
            #         position=math.nan,
            #         kp_scale=0.0,
            #         kd_scale=0.0,
            #         feedforward_torque=-torque,
            #         maximum_torque=0.1,
            #         query=True,
            #     )
            # ]

            request_start_time = time.time()
            # task = asyncio.create_task(fdcanusb_transport.cycle(commands))
            # tasks.append(task)
            request_end_time = time.time()

            query_commands = [
                motors[1].make_query(),
                motors[4].make_query()
            ]
            query_start = time.time()
            query_task = asyncio.create_task(fdcanusb_transport.cycle(query_commands))
            query_end = time.time()


            # feedback_1 = await motor1.set_torque(torque)
            # request1_end_time = time.time()
            # # await asyncio.sleep(0.03)  # Small delay to prevent overwhelming the motors
            # feedback_2 = await motor4.set_torque(torque)
            # request2_end_time = time.time()
            
            
            # request1_times.append(request1_end_time - request_start_time)
            # request2_times.append(request2_end_time - request1_end_time)
            # total_request_times.append(request2_end_time - request_start_time)

            imu_start_time = time.time()
            imudata = td.IMUData(*imu.read_accelerometer_gyro(convert=True))
            imu_end_time = time.time()
            euler_angles = td.EulerAngles(
                *sensor_fusion.update(
                    imudata.get_gyro(), imudata.get_accel(), delta_time=0.01
                )[0]
            )
            fusion_end_time = time.time()

            # wait on query task
            query_return = await query_task

            request_times.append(request_end_time - request_start_time)
            query_times.append(query_end - query_start)
            imu_times.append(imu_end_time - imu_start_time)
            fusion_times.append(fusion_end_time - imu_end_time)

            while(time.time() - loop_start_time < 0.01):
                await asyncio.sleep(0.01)  # Small delay to prevent overwhelming the motors

            print(query_return)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping motors...")
    finally:
        # await motor1.stop()
        # await motor4.stop()
        await fdcanusb_transport.cycle([motor.make_stop() for motor in motors.values()])
        # print(np.mean(request_times))
        print(np.mean(query_times))
        print(np.mean(imu_times))
        print(np.mean(fusion_times))

        with open("transport_final_request_times.csv", "w") as final_file:
            final_file.write("request_times\n")
            for r in request_times:
                final_file.write(f"{r}\n")
            # final_file.write("request1_times,request2_times,total_request_times\n")
            # for r1, r2, tr in zip(request1_times, request2_times, total_request_times):
            #     final_file.write(f"{r1},{r2},{tr}\n")


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exited... Double check that the motors are off and not ringing!")





