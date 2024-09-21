import asyncio
import datetime
import math
import time

import moteus

"""
==========================
Position control arguments
==========================
  def make_position(self,
                      *,
                      position=None,
                      velocity=None,
                      feedforward_torque=None,
                      kp_scale=None,
                      kd_scale=None,
                      maximum_torque=None,
                      stop_position=None,
                      watchdog_timeout=None,
                      velocity_limit=None,
                      accel_limit=None,
                      fixed_voltage_override=None,
                      query=False,
                      query_override=None):
"""

TORQUE = 0.03
MAX_VEL = 0.1

async def main(c):
  await c.set_stop() # clear faults
  while True:
        current_command = 0.03 # N*m
        results = await c.set_position(
            position=math.nan,
            kp_scale=0.0,
            kd_scale=0.0,
            feedforward_torque=TORQUE,
            maximum_torque=0.3,
            velocity_limit=MAX_VEL,     
            query=True,
        )
        await asyncio.sleep(0.02)

async def stop(c):
    await c.set_stop()

if __name__ == '__main__':
    c = moteus.Controller(transport=None)
    try: 
        asyncio.run(main(c))
    except KeyboardInterrupt: 
        print("Exiting...")
        asyncio.run(stop(c))
        f.close()