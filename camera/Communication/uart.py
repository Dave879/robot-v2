# Untitled - By: devkosu - Sun Feb 19 2023

import sensor, image, time
from pyb import UART

uart = UART(3)

uart.init(baudrate=115200, timeout_char=1000)

clock = time.clock()

while True:
    uart.writechar(49)

    time.sleep(1)

    print(uart.any())
