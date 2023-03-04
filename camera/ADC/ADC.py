# Untitled - By: devkosu - Sat Mar 4 2023

import sensor, image, time

import pyb

adc = pyb.ADC(pyb.Pin('P6'))                  # create an analog object from a pin

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.B64X64)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    # clock.tick()
    img = sensor.snapshot()
    print("%f volts" % (((adc.read() * 3.3) + 2047.5) / 4095)) # read value, 0-4095
    # print(clock.fps())
