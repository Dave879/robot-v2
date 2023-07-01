import sensor, image, time, math, os, tf, uos, gc

from pyb import LED

from pyb import UART

uart = UART(3)

uart.init(baudrate=115200, timeout_char=1)

# COLOR DETECTION

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/black/yellow things.
red =(10, 100, 20, 127, 0, 127) # generic_red_thresholds
green =(10, 50, -128, -20, -1, 127) # generic_green_thresholds
yellow=(20, 100, -15, 10, 20, 127) # generic_yellow_thresholds
black=(0, 15, -10, 10, -10, 10) # generic_black_thresholds

pixels_threshold = 80
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

time.sleep(1)

sensor.reset()                         # Reset and initialize the sensor.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_auto_exposure(False)
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565
sensor.set_framesize(sensor.QQVGA)      # Set frame size to QVGA (160x160)
sensor.skip_frames(time=2000)          # Let the camera adjust.
img2 = sensor.alloc_extra_fb(sensor.width(), sensor.width(), sensor.GRAYSCALE)


red_led = LED(1)
blue_led = LED(3)
green_led = LED(2)

while(True):
    red_led.on()
    img = sensor.snapshot()
    if uart.any():
        blue_led.on()
        data = uart.read().decode('utf-8').rstrip()
        print(data)
        # Requesting teensy 4.1 to verify that openmv is up and running
        #if data == '0':
            # Send a confirmation message to teensy 4.1
        #    uart.writechar(5)
        # Signal to start the search for victims
        if data == '9':
            print("OpenMV inizia a cercare...")
            red_led.off()
            need_to_stop = False
            img = sensor.snapshot()
            for i in thresholds:
                for blob in img.find_blobs([i], pixels_threshold=pixels_threshold, area_threshold=area_threshold):
                    if blob.cy() < 32 or blob.cy() > 100:
                        continue
                    if i == black:
                        print("Black")
                        need_to_stop = True
                    elif i == green:
                        print("green")
                        uart.writechar(0 + 48) # Nubmer of kits
                    elif i == yellow or red:
                        uart.writechar(1 + 48)
                        if i == yellow:
                            print("Yellow")
                        else:
                            print("Red")

            if need_to_stop:
                uart.writechar(9 + 48) # Teel teensy 4.1 if there are black victims
            else:
                blue_led.off()
                continue

            while not uart.any():
                pass
            data = uart.read().decode('utf-8').rstrip()
            if  data != '8':
                continue
            #mg.lens_corr(1.8)
            img = sensor.snapshot()
            img2.draw_rectangle(0,0, img2.width(), img2.height(), (255,255,255), fill=True)
            img2.draw_image(img, 0 ,math.floor(img2.height()/2 - img.height()/2))

            for i, detection_list in enumerate(net.detect(img2, thresholds=[(math.ceil(min_confidence * 255), 255)])):
                if (i == 0): continue # background class
                if (len(detection_list) == 0): continue # no detections for this class?

                print("********** %s **********" % labels[i])
                for d in detection_list:
                    [x, y, w, h] = d.rect()
                    center_x = math.floor(x + (w / 2))
                    center_y = math.floor(y + (h / 2))
                    print('x %d\ty %d' % (center_x, center_y))
                    img2.draw_circle((center_x, center_y, 12), color=colors[i], thickness=2)
                    if labels[i] == "H":
                        uart.writechar(3 + 48)
                    elif labels[i] == "S":
                        uart.writechar(2 + 48)
                    elif labels[i] == "U":
                        uart.writechar(0 + 48)

        blue_led.off()
