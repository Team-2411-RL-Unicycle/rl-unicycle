import asyncio
import math
import moteus
import time
import numpy as np


async def main():
    fdcanusb_transport = moteus.Fdcanusb(
        "/dev/serial/by-id/usb-mjbots_fdcanusb_5A70499D-if00"
    )

    motors = {
        motor_id: moteus.Controller(id=motor_id, transport=fdcanusb_transport)
        for motor_id in [4, 5, 6]
    }

    await fdcanusb_transport.cycle([x.make_stop() for x in motors.values()])

    request_times = []

    try:
        while True:
            loop_start = time.time()

            commands = [
                motors[4].make_position(
                    position=math.nan, velocity=0.1 * math.sin(loop_start), query=True
                ),
                motors[5].make_position(
                    position=math.nan,
                    velocity=0.1 * math.sin(loop_start + 1),
                    query=True,
                ),
                motors[6].make_position(
                    position=math.nan,
                    velocity=0.1 * math.sin(loop_start + 2),
                    query=True,
                ),
            ]

            await fdcanusb_transport.cycle(commands)
            request_time = time.time()
            request_times.append(request_time - loop_start)

            while time.time() - loop_start < 0.01:
                await asyncio.sleep(0.01)
    except asyncio.CancelledError:
        print("Task was canceled. Performing cleanup...")
        raise  # Ensures the outer loop can manage cancellation properly
    finally:
        await fdcanusb_transport.cycle([motor.make_stop() for motor in motors.values()])
        print("Motors stopped.")
        if request_times:
            print("Average request time:", np.mean(request_times))
        with open("transport_final_request_times.csv", "w") as final_file:
            final_file.write("request_times\n")
            for r in request_times:
                final_file.write(f"{r}\n")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        task = loop.create_task(main())
        loop.run_until_complete(task)
    except KeyboardInterrupt:
        print("Main loop interrupted. Cleaning up...")
        # Cancel all running tasks
        for task in asyncio.all_tasks(loop):
            task.cancel()
        loop.run_until_complete(
            asyncio.gather(*asyncio.all_tasks(loop), return_exceptions=True)
        )
    finally:
        print("Exiting... Double check that the motors are off and not ringing!")
        loop.close()
