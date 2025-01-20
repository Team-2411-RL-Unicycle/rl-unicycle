import asyncio
import math
import moteus
import time
import numpy as np


async def main():
    fdcanusb_transport = moteus.Fdcanusb(
        '/dev/serial/by-id/usb-mjbots_fdcanusb_B1457958-if00')

    # We create one 'moteus.Controller' instance for each servo.  It
    # is not strictly required to pass a 'transport' since we do not
    # intend to use any 'set_*' methods, but it doesn't hurt.
    #
    # This syntax is a python "dictionary comprehension":
    # https://docs.python.org/3/tutorial/datastructures.html#dictionaries
    motors = {
        motor_id: moteus.Controller(id=motor_id, transport=fdcanusb_transport)
        for motor_id in [4, 5, 6]
    }

    # We will start by sending a 'stop' to all servos, in the event
    # that any had a fault.
    await fdcanusb_transport.cycle([x.make_stop() for x in motors.values()])

    request_times = []

    try:
        while True:
            # The 'cycle' method accepts a list of commands, each of which
            # is created by calling one of the `make_foo` methods on
            # Controller.  The most common thing will be the
            # `make_position` method.

            loop_start = time.time()

            # For now, we will just construct a position command for each
            # of the 4 servos, each of which consists of a sinusoidal
            # velocity command starting from wherever the servo was at to
            # begin with.
            #
            # 'make_position' accepts optional keyword arguments that
            # correspond to each of the available position mode registers
            # in the moteus reference manual.
            commands = [
                motors[4].make_position(
                    position=math.nan,
                    velocity=0.1*math.sin(loop_start),
                    query=True),
                motors[5].make_position(
                    position=math.nan,
                    velocity=0.1*math.sin(loop_start + 1),
                    query=True),
                motors[6].make_position(
                    position=math.nan,
                    velocity=0.1*math.sin(loop_start + 2),
                    query=True),
            ]

            # By sending all commands to the transport in one go, the
            # pi3hat can send out commands and retrieve responses
            # simultaneously from all ports.  It can also pipeline
            # commands and responses for multiple servos on the same bus.
            await fdcanusb_transport.cycle(commands)
            request_time = time.time()

            request_times.append(request_time - loop_start)

            # The result is a list of 'moteus.Result' types, each of which
            # identifies the servo it came from, and has a 'values' field
            # that allows access to individual register results.
            #
            # Note: It is possible to not receive responses from all
            # servos for which a query was requested.
            #
            # Here, we'll just print the ID, position, and velocity of
            # each servo for which a reply was returned.
            # print(", ".join(
            #     f"({result.arbitration_id} " +
            #     f"{result.values[moteus.Register.POSITION]} " +
            #     f"{result.values[moteus.Register.VELOCITY]})"
            #     for result in results))

            # We will wait 20ms between cycles.  By default, each servo
            # has a watchdog timeout, where if no CAN command is received
            # for 100ms the controller will enter a latched fault state.
            while (time.time() - loop_start < 0.01):
                # Small delay to prevent overwhelming the motors
                await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping motors...")
    finally:
        # await motor1.stop()
        # await motor4.stop()
        await fdcanusb_transport.cycle([motor.make_stop() for motor in motors.values()])
        print(np.mean(request_times))

        with open("transport_final_request_times.csv", "w") as final_file:
            final_file.write("request_times\n")
            for r in request_times:
                final_file.write(f"{r}\n")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exited... Double check that the motors are off and not ringing!")
