# Untitled - By: devkosu - Sun Feb 19 2023

import sensor, time
from pyb import UART

uart = UART(3)

uart.init(baudrate=115200, timeout_char=1000)

while True:

    # uart.writechar(49)

    # time.sleep(1)

    # print(uart.any())

    print(uart.any())
    if uart.any() > 0:
        data = uart.read()
        print(data)
