import cv2
import numpy as np
from colors import BGR_COMMON


def filter_dummy(frame, config):
    return frame


def filter_overlay(detected_objects, frame, config):
    frame = frame.copy()
    q = detected_objects
    while not q.empty():
        item = q.get()
        if item is None:
            continue
        print(item)

        cv2.circle(frame, (int(item["x"]-1),int(item["y"]-1)), 2, BGR_COMMON[item['colour']], 1)
        print q.get() # this queue can be used to draw the objects on the overlay.
    return frame


def filter_normalize(frame, config):
    normalizing_factors = np.sum(frame, axis=2)
    # avoid division by zero
    normalizing_factors[normalizing_factors == 0] = 1
    h, w = normalizing_factors.shape

    # Now we want to divide elementwise (which numpy allows us to do quite nicely)
    # But in order for broadcasting to work we need the normalizing_factor tensor3
    # to have the same order as the china tensor

    norm = frame.astype(float) / normalizing_factors.reshape(h, w, 1)
    norm *= 255
    norm = norm.astype(np.uint8)

    return norm


def filter_grayscale(frame, config):
    frame = np.sum(frame, axis=2)
    h, w = frame.shape
    normalizing_factors = np.empty(h * w)
    normalizing_factors.fill(3 * 255)

    # Now we want to divide elementwise (which numpy allows us to do quite nicely)
    # But in order for broadcasting to work we need the normalizing_factor tensor3
    # to have the same order as the china tensor

    out = frame.astype(float) / normalizing_factors.reshape(h, w)
    out *= 255
    out = out.astype(np.uint8)

    return np.repeat(out[:, :, np.newaxis], 3, axis=2)


def filter_colour(colour, frame, config):
    # Convert BGR to HSV
    bgr=frame
    bgr_low=config.colours[colour]['min']
    bgr_high=config.colours[colour]['max']

    # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # hsv_low = cv2.cvtColor(np.uint8([[bgr_low]]), cv2.COLOR_BGR2HSV)[0][0]
    # hsv_high = cv2.cvtColor(np.uint8([[bgr_high]]), cv2.COLOR_BGR2HSV)[0][0]

    # ([50, 100, 100])
    # 70, 255, 255]

    # Threshold the HSV image to get only blue colors
    # mask = cv2.inRange(hsv, lower_green, upper_green)
    mask_bgr = cv2.inRange(bgr, bgr_low, bgr_high)
    # mask_hsv = cv2.inRange(hsv, hsv_low, hsv_high)
    # Bitwise-AND mask and original image

    mask = mask_bgr

    res = cv2.bitwise_and(frame, frame, mask=mask)

    return res
