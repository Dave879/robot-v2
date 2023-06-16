import sensor, image, time

from pyb import UART


# UART COMMUNICATION

uart = UART(3)

uart.init(baudrate=115200, timeout_char=1)

# CAMERA

sensor.reset()                      
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
time.sleep(0.01)

i = 0

uart.write("Ciao da openMV")

while(True):
  uart.write(str(i))
  i += 1
  time.sleep(0.5)
