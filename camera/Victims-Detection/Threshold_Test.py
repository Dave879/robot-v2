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
green=(10, 50, -128, -20, -1, 127) # green_thresholds
yellow=(20, 65, -15, 30, 40, 127) # yellow_thresholds
black=(0, 15, -10, 10, -10, 10) # black_thresholds


pixels_threshold = 50
area_threshold = 50

thresholds = [red, green, yellow, black]


# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. Don't set "merge=True" becuase that will merge blobs which we don't want here.

# AI
net = None
labels = None
min_confidence = 0.5

try:
    # load the model, alloc the model file on the heap if we have at least 64K free after loading
    net = tf.load("trained.tflite", load_to_fb=uos.stat('trained.tflite')[6] > (gc.mem_free() - (64*1024)))
except Exception as e:
    raise Exception('Failed to load "trained.tflite", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

try:
    labels = [line.rstrip('\n') for line in open("labels.txt")]
except Exception as e:
    raise Exception('Failed to load "labels.txt", did you copy the .tflite and labels.txt file onto the mass-storage device? (' + str(e) + ')')

colors = [ # Add more colors if you are detecting more than 7 types of classes at once.
    (255,   0,   0),
    (  0, 255,   0),
    (255, 255,   0),
    (  0,   0, 255),
    (255,   0, 255),
    (  0, 255, 255),
    (255, 255, 255),
]

# CAMERA

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.B128X64)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
time.sleep(0.01)
#sensor.set_auto_gain(True) # must be turned off for color tracking
#sensor.set_auto_whitebal(True) # must be turned off for color tracking


# Number of kits to drop : 0 kit(is not the same as -1), 1 kit, 2 kits or 3 kits
kits = -1 # -1 means that there is no victim

while(True):

    sharp_read = analog_distance.read()

    print("%f <- value" % sharp_read) # read value, 0-4095


    img = sensor.snapshot() # Take a picture and return the image.

    #img.negate()

    for i in thresholds:

        for blob in img.find_blobs([i], pixels_threshold=pixels_threshold, area_threshold=area_threshold):

                # These values depend on the blob not being circular - otherwise they will be shaky.
                #if blob.elongation() > 0.5: # TODO: test with all letters to get the value all leters are detected with
                    #img.draw_edges(blob.min_corners(), color=(255,0,0))
                    #img.draw_line(blob.major_axis_line(), color=(0,255,0))
                    #img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                # These values are stable all the time.
                #img.draw_rectangle(blob.rect())
                #img.draw_cross(blob.cx(), blob.cy())
                # Note - the blob rotation is unique to 0-180 only.
                #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

                # Black detected
                if i == black:
                    sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
                    img = sensor.snapshot()

                    for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
                        if (i == 0): continue # background class
                        if (len(detection_list) == 0): continue # no detections for this class?

                        print("********** %s **********" % labels[i])
                        for d in detection_list:
                            [x, y, w, h] = d.rect()
                            center_x = math.floor(x + (w / 2))
                            center_y = math.floor(y + (h / 2))
                            print('x %d\ty %d' % (center_x, center_y))
                            img.draw_circle((center_x, center_y, 12), color=colors[i], thickness=2)

                        if labels[i]== "H":
                            print("black")
                            kits = 3
                        elif labels[i] == "S":
                            print("black")
                            kits = 2
                        elif labels[i]== "U":
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
        green_led = LED(2)
        kits =-1
        # Lampeggio -> Vittima
