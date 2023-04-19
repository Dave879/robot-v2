# Grayscale Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import sensor, image, time

sensor.reset()
sensor.set_framesize(sensor.B64X64)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.skip_frames(time = 2000)
clock = time.clock()

low_threshold = (0, 20)

while(True):


    # Test not low threshold
    for i in range(100):
        clock.tick()
        img = sensor.snapshot()
        img.binary([low_threshold], invert = 1)
        print(clock.fps())
