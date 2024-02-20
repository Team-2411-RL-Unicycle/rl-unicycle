## Jetson IO Cofiguration

The 40-pin header needs to be configured to allow for SPI communication with the IMUs:

https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-325/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/hw_setup_jetson_io.html

Linux for Tegra version on Jetson as of 2024-01-30 is 32.7.4

**R32 (release), REVISION: 7.4, GCID: 33514132, BOARD: t210ref, EABI: aarch64, DATE: Fri Jun  9 04:25:08 UTC 2023**

>[!Note]
> The Jetson IO configuration is handled using a Python script that can be run using the command `sudo /opt/nvidia/jetson-io/jetson-io.py` in the terminal. 

The full stack of communicationo pins and pwm pins can be accessed through this menu using the commands:

- Configure Jetson 40pin Header
    - Configure header pins manually
        - spi1 (19,21,23,24,26)
        - spi2 (13,16,18,22,37)

Each SPI port has a clock, data in, data out, and two CS pins allowing for multiplexing two devices. These CS pins can be expanded further by using manual definitions on some of the other IO pins.

Both SPI ports have been turned on, the overall pin-mapping is:

|      Name | Pin | Pin | Name      |
|----------:|----:|:----|:----------|
|      3.3V |   1 | 2   | 5V        |
|      i2c2 |   3 | 4   | 5V        |
|      i2c2 |   5 | 6   | GND       |
|    unused |   7 | 8   | uartb     |
|       GND |   9 | 10  | uartb     |
|    unused |  11 | 12  | unused    |
|   spi2_sck|  13 | 14  | GND       |
|        NA |  15 | 16  | spi2_cs1  |
|      3.3V |  17 | 18  | spi2_cs0  |
| spi1_dout |  19 | 20  | GND       |
|  spi1_din |  21 | 22  | spi2_din  |
|  spi1_sck |  23 | 24  | spi1_cs0  |
|       GND |  25 | 26  | spi1_cs1  |
|      i2c1 |  27 | 28  | i2c1      |
|        NA |  29 | 30  | GND       |
|        NA |  31 | 32  | unused    |
|    unused |  33 | 34  | GND       |
|    unused |  35 | 36  | unused    |
| spi2_dout |  37 | 38  | unused    |
|       GND |  39 | 40  | unused    |

For SPI2 The pins are:
13 - SCK (Clock)
22 - MISO (Data In)
37 - MOSI (Data Out)
16 - CS0 (Chip Select 0)
18 - CS1 (Chip Select 1)

### SPI Testing

sudo modprobe spidev
ls /dev/spi*

This should open the SPI devices (0 indexed) so that SPI 1 shows as 0 and SPI 2 shows as 1. The ls command will list the devices showing that they are in fact open.

To test the pins spidev for python is installed, but python3 is missing. 

sudo apt install python3 python3-venv

from in the project directory:

python3 -m venv venv
source venv/bin/activate

SPI is non functioning at this time.

## I2C

The imu can be read through i2c, it is wired to a bus 




