import sensor, image, time, math, os, tf, uos, gc

# COLOR DETECTION

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/black/yellow things.
thresholds = [(0, 60, 15, 127, 15, 127), # generic_red_thresholds
              (20, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 10, -30, 10, -20, 30),   # generic_black_thresholds
              (30, 100, -10, 10, 15, 127)] # generic_yellow_thresholds


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
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

while(True):
    img = sensor.snapshot()         # Take a picture and return the image.

    for i in thresholds:
        for blob in img.find_blobs([i], pixels_threshold=200, area_threshold=200):
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
            if i == (0, 10, -30, 10, -20, 30):
                # sensor.reset()                      # Reset and initialize the sensor.
                sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
                #sensor.skip_frames(time = 2000)     # Wait for settings take effect.
                # default settings just do one detection... change them to search the image...
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
                    # sensor.reset()                      # Reset and initialize the sensor.
                    sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
                    #sensor.skip_frames(time = 2000)     # Wait for settings take effect.
