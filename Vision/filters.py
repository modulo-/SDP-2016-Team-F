import copy
import cmath
import cv2
import math
import numpy as np
from colors import BGR_COMMON


def filter_dummy(frame, config):
    return frame


def filter_overlay(world, oldworld, frame, config):
    if world.ball is not None:
        ball = world.ball

        cv2.circle(frame, (int(ball.x-3),int(ball.y-3)), 6, BGR_COMMON[ball['colour']], 3)

        length = ball.velocity
        complex = cmath.rect(length, ball.angle)
        y = -complex.real
        x = complex.imag

        arrowhead = (int(x+world.ball.x), int(y+world.ball.y))
        cv2.arrowedLine(frame, ball.centre, arrowhead, BGR_COMMON['red'], 2, cv2.CV_AA)

    for team in ['blue', 'yellow']:
        for colour in ['pink', 'green']:
            robot = getattr(world, "robot_{}_{}".format(team, colour))
            if robot is None:
                continue

            cv2.circle(frame, (int(robot.x-5),int(robot.y-3)), 10, BGR_COMMON[colour], 3)

            length = 15
            complex = cmath.rect(length, robot.angle)
            y = -complex.real
            x = complex.imag

            arrowhead = (int(x + robot.x), int(y+robot.y))
            cv2.arrowedLine(frame, robot.centre, arrowhead, BGR_COMMON['black'], 2, cv2.CV_AA)



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
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if (colour is None):
        hsv_low=np.array(config.low)
        hsv_high=np.array(config.high)
    else:
        hsv_low = np.array(config.colours[colour]["min"]).astype(np.uint8)
        hsv_high = np.array(config.colours[colour]["max"]).astype(np.uint8)

    # ([50, 100, 100])
    # 70, 255, 255]

    # print(bgr_high, bgr_low)
    # Threshold the HSV image to get only blue colors
    # mask = cv2.inRange(hsv, lower_green, upper_green)
    # mask_bgr = cv2.inRange(bgr, bgr_low, bgr_high)
    mask_hsv = cv2.inRange(hsv, hsv_low, hsv_high)
    # Bitwise-AND mask and original image
    frame_mask = mask_hsv

    adjustments = {
        "open":config.open,
        "close":config.close,
        "erode":config.erode,
        "dilate":config.dilate
    }

    if adjustments['open'] >= 1:
        kernel = np.ones((2,2),np.uint8)
        frame_mask = cv2.morphologyEx(frame_mask,
                                      cv2.MORPH_OPEN,
                                      kernel,
                                      iterations=adjustments['open'])

    if adjustments['close'] >= 1:
        kernel = np.ones((2,2),np.uint8)
        frame_mask = cv2.dilate(frame_mask,
                                kernel,
                                iterations=adjustments['close'])

    if adjustments['erode'] >= 1:
        kernel = np.ones((2,2),np.uint8)
        frame_mask = cv2.erode(frame_mask,
                                kernel,
                                iterations=adjustments['erode'])

    if adjustments['dilate'] >= 1:
        kernel = np.ones((2,2),np.uint8)
        frame_mask = cv2.dilate(frame_mask,
                                kernel,
                                iterations=adjustments['dilate'])

    res = cv2.bitwise_and(hsv, hsv, mask=frame_mask)
    res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

    return res
