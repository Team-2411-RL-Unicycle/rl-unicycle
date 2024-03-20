import asyncio
import math
import moteus
import time
import datetime

async def main(c,f):
  await c.set_stop() # clear faults
  while True:
        current_command = 5 if (round(time.time()) % 2) else -5
        results = await c.set_position(
            position=current_command,
            velocity=0.0,
            accel_limit=8.0,
            velocity_limit=3.0,
            query=True,
        )
        results = {item.split(':')[0].strip(): float(item.split(':')[1].strip()) for item in str(results).replace('}','').split(',')[1:]}
        results = f'{int(time.time()*1000)}: {results}\n'
        print(results)
        f.write(results)
        await asyncio.sleep(0.02)



if __name__ == '__main__':
    c = moteus.Controller(transport=None)
    f = open(f'logs/{datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}.txt', 'x')
    try: 
        asyncio.run(main(c,f))
    except KeyboardInterrupt: 
        print("Exiting...")
        c.set_stop()
        f.close()