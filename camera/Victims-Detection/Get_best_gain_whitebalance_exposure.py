# Sensor Manual Gain Control
#
# This example shows off how to control the camera sensor's
# gain manually versus letting auto gain control run.

# What's the difference between gain and exposure control?
#
# Well, by increasing the exposure time for the image you're getting more
# light on the camera. This gives you the best signal to noise ratio. You
# in general always want to increase the expsoure time... except, when you
# increase the exposure time you decrease the maximum possible frame rate
# and if anything moves in the image it will start to blur more with a
# higher exposure time. Gain control allows you to increase the output per
# pixel using analog and digital multipliers... however, it also amplifies
# noise. So, it's best to let the exposure increase as much as possible
# and then use gain control to make up any remaining ground.

import sensor, image, time

# Change this value to adjust the gain. Try 10.0/0/0.1/etc.
GAIN_SCALE = 1.0

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)

# Print out the initial gain for comparison.
print("Initial gain == %f db" % sensor.get_gain_db())

sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# You have to turn automatic exposure control and automatic white blance off
# otherwise they will change the image exposure to undo any gain settings
# that you put in place...
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(True)
# Need to let the above settings get in...
sensor.skip_frames(time = 500)

while(True):
    img = sensor.snapshot()         # Take a picture and return the image.
    print("Exposure == %f" % sensor.get_exposure_us())
    print("Gain == %f" % sensor.get_gain_db())
    print("White balance == " + str(sensor.get_rgb_gain_db()))
