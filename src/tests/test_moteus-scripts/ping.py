import asyncio
import math
import moteus


async def main(c):

    await c.set_stop()

    while True:
        state = await c.set_position(position=math.nan, query=True)

        print(state)

        print("Position", state.values[moteus.Register.POSITION])

        print()

        # note that the moteus requires there to be a command
        # at least every 100ms or the controller will fault
        await asyncio.sleep(0.05)


async def stop(c):
    await c.set_stop()


if __name__ == "__main__":
    c = moteus.Controller()
    try:
        asyncio.run(main(c))
    except KeyboardInterrupt:
        asyncio.run(stop(c))
        print("Exiting...")
