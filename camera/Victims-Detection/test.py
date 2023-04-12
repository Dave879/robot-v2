import sensor, image, time, math, os, tf, uos, gc

from pyb import LED

from pyb import UART

import pyb


# ANALOG READ

analog_distance = pyb.ADC(pyb.Pin('P6'))


# UART COMMUNICATION

uart = UART(3)

uart.init(baudrate=115200, timeout_char=1)

# COLOR DETECTION

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/black/yellow things.
red=(10, 100, 20, 127, 0, 127) # red_thresholds
green=(10, 50, -128, -25, -1, 127) # green_thresholds
yellow=(20, 70, -15, 30, 25, 127) # yellow_thresholds
black=(0, 4, -20, 20, -20, 20) # black_thresholds

pixels_threshold = 50
area_threshold = 50

thresholds = [red, green, yellow, black]


# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.

# AI

net = None
labels = None

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    print(e)
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

# CAMERA

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.B64X64)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
time.sleep(0.01)
#sensor.set_auto_gain(True) # must be turned off for color tracking
#sensor.set_auto_whitebal(True) # must be turned off for color tracking


# Number of kits to drop : 0 kit(is not the same as -1), 1 kit, 2 kits or 3 kits
kits = -1 # -1 means that there is no victim

while(True):

    if uart.any():
        data = uart.read().decode('utf-8').rstrip()
        if data == '9':
            kits = -1

    if kits != -1:
        continue

    sharp_read = analog_distance.read()

    print("%f <- value" % sharp_read) # read value, 0-4095

    if (sharp_read > 350):

        img = sensor.snapshot() # Take a picture and return the image.

        for i in thresholds:
            pixels_threshold = int(sharp_read * 0.17)
            area_threshold= int(sharp_read * 0.17)

            for blob in img.find_blobs([i], pixels_threshold=pixels_threshold, area_threshold=area_threshold):

                    # These values depend on the blob not being circular - otherwise they will be shaky.
                    if blob.elongation() > 0.5: # TODO: test with all letters to get the value all leters are detected with
                        img.draw_edges(blob.min_corners(), color=(255,0,0))
                        img.draw_line(blob.major_axis_line(), color=(0,255,0))
                        img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                    # These values are stable all the time.
                    img.draw_rectangle(blob.rect())
                    img.draw_cross(blob.cx(), blob.cy())
                    # Note - the blob rotation is unique to 0-180 only.
                    img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

                    # Black detected
                    if i == black:
                        sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
                        img = sensor.snapshot()

                        for obj in net.classify(img, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5):
                            print("**********\nPredictions at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
                            img.draw_rectangle(obj.rect())
                            # This combines the labels and confidence values into a list of tuples
                            predictions_list = list(zip(labels, obj.output()))

                            letter_max_value = 0
                            lable = ""
                            for i in range(len(predictions_list)):
                                    if predictions_list[i][1] > letter_max_value:
                                        letter_max_value = predictions_list[i][1]
                                        lable = predictions_list[i][0]
                                    # print("%s = %f" % (predictions_list[i][0], predictions_list[i][1]))

                            print("Lettera: %s con %f" %(lable, letter_max_value))
                            if lable == "H":
                                print("black")
                                kits = 3
                            elif lable == "S":
                                print("black")
                                kits = 2
                            elif lable == "U":
                                print("black")
                                kits = 0

                            sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)

                    # Red detected
                    elif i == red:
                        print("red")
                        kits = 1
                    # Yellow detected
                    elif i == yellow:
                        print("yellow")
                        kits = 1
                    # Green detected
                    elif i == green:
                        print("green")
                        kits = 0

        if kits >= 0:
            print(kits)
            send = kits + 48
            uart.writechar(send)
