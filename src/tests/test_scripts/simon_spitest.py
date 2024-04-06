import spidev
import time

# Create an instance of the SPI class
spi = spidev.SpiDev()

# Open the SPI device. This corresponds to the device files in /dev
SPI_BUS = 1
SPI_DEVICE = 0
spi.open(SPI_BUS, SPI_DEVICE)  # Open the SPI bus

# Set SPI speed and mode
spi.max_speed_hz = 10000
spi.mode = 0

try:
    while True:
        resp = spi.xfer([0xAA])  # Send a single byte
        print("Response: ", resp)
        time.sleep(1)
except KeyboardInterrupt:
    spi.close()
