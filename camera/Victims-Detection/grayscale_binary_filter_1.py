# Grayscale Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import sensor, image, time

sensor.reset()
sensor.set_framesize(sensor.QQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.skip_frames(time = 2000)
clock = time.clock()

low_threshold = (0, 60)

while(True):


    # Test not low threshold
    for i in range(100):
        clock.tick()
        img = sensor.snapshot()
        img.histeq(True)
        img.median(True)
        img.binary([low_threshold], invert = 1)
        img.draw_rectangle(0,img.height()-30,img.width(), 30, (255,255,255), 1, True)
        print(clock.fps())
