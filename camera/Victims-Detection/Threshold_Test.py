import sensor, image, time, math, os, tf, uos, gc

from pyb import LED

# COLOR DETECTION

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/black/yellow things.
red =(10, 100, 20, 127, 10, 127) # generic_red_thresholds
green =(5, 100, -128, -15, 10, 127) # generic_green_thresholds
yellow=(20, 100, -10, 20, 20, 127) # generic_yellow_thresholds
black=(0, 15, -128, 127, -128, 127) # generic_black_thresholds

pixels_threshold = 100
area_threshold = 300

black_pixels_threshold = 10
black_area_threshold = 90

thresholds = [green, yellow, red]

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
sensor.set_pixformat(sensor.RGB565)    # Set pixel format to RGB565
sensor.set_framesize(sensor.QQVGA)      # Set frame size to QVGA (160x160)
sensor.skip_frames(time=2000)          # Let the camera adjust.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_auto_exposure(False)
# Need to let the above settings get in...
sensor.skip_frames(time = 500)

# Auto gain control (AGC) is enabled by default. Calling the below function
# disables sensor auto gain control. The additionally "gain_db"
# argument then overrides the auto gain value after AGC is disabled.
sensor.set_auto_gain(False, \
    gain_db = 1.02222)

img2 = sensor.alloc_extra_fb(sensor.width(), sensor.width(), sensor.GRAYSCALE)

red_led = LED(1)
blue_led = LED(3)
green_led = LED(2)

while(True):
    img = sensor.snapshot()
    img.draw_rectangle(0,img.height()-25,img.width(), 25, (255,255,255), 1, True)
    img.draw_rectangle(0,0,img.width(), 18, (255,255,255), 1, True)
    img.draw_circle(15, 9, 27, (255,255,255), 1, True)
    img.draw_circle(150, 7, 27, (255,255,255), 1, True)

    for i in thresholds:
        for blob in img.find_blobs([i], pixels_threshold=pixels_threshold, area_threshold=area_threshold):
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())

            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

            if i == green:
                print("Green")
            elif i == yellow:
                print("Yellow")
            elif i == red:
                print("Red")

    img = sensor.snapshot()
    img.histeq(adaptive=True, clip_limit=3)
    #img.median(True)
    img.draw_rectangle(0,img.height()-25,img.width(), 25, (255,255,255), 1, True)
    img.draw_rectangle(0,0,img.width(), 18, (255,255,255), 1, True)
    img.draw_circle(15, 9, 27, (255,255,255), 1, True)
    img.draw_circle(150, 7, 27, (255,255,255), 1, True)
    for blob in img.find_blobs([black], pixels_threshold=black_pixels_threshold, area_threshold=black_area_threshold):
        print("Black")
        img = sensor.snapshot()
        img.histeq(True)
        img.median(True)
        img.draw_rectangle(0,img.height()-25,img.width(), 25, (255,255,255), 1, True)
        img.draw_rectangle(0,0,img.width(), 18, (255,255,255), 1, True)
        img.draw_circle(15, 9, 27, (255,255,255), 1, True)
        img.draw_circle(150, 7, 27, (255,255,255), 1, True)
        #img.lens_corr(1.8)
        img2.draw_rectangle(0,0, img2.width(), img2.height(), (255,255,255), fill=True)
        img2.draw_image(img, 0 ,math.floor(img2.height()/2 - img.height()/2))

        # detect() returns all objects found in the image (splitted out per class already)
        # we skip class index 0, as that is the background, and then draw circles of the center
        # of our objects

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
