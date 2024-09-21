import asyncio
import math
import time

import moteus


async def main():
    c = moteus.Controller(transport=None)
    # In case the controller had faulted previously, at the start of
    # this script we send the stop command in order to clear it.
    await c.set_stop()  # clear faults


if __name__ == "__main__":
    asyncio.run(main())
