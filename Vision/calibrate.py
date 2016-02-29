from collections import OrderedDict
import cv2
import numpy as np
import time
import oldvision.tools as tools
import argparse

from oldvision.vision import _Vision

FRAME_NAME = 'ConfigureWindow'

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)

distort_data = tools.get_radial_data()
NCMATRIX = distort_data['new_camera_matrix']
CMATRIX = distort_data['camera_matrix']
DIST = distort_data['dist']


class Calibrate(object):
    def __init__(self, cam, config, width=640, height=480):
        self.config = config
        self.width = width
        self.height = height
        self.pitch = config.pitch_room.selected
        self.camera = cam
        self.new_polygon = True
        self.polygon = self.polygons = []
        self.points = []

        keys = ['outline', 'Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']
        self.data = self.drawing = {}

        # Create keys
        for key in keys:
            self.data[key] = []
            self.drawing[key] = []

        self.color = RED


    def run(self, camera=False):
        frame = cv2.namedWindow(FRAME_NAME)

        # Set callback
        cv2.setMouseCallback(FRAME_NAME, self.draw)

        if camera:
            cap = cv2.VideoCapture(0)
            for i in range(10):
                status, image = cap.read()
        else:
            image = cv2.imread('00000001.jpg')

        self.image = cv2.undistort(image, CMATRIX, DIST, None, NCMATRIX)

        # Get various data about the image from the user
        self.get_pitch_outline()

        print 'Press any key to finish.'
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Write out the data
        # self.dump('calibrations/calibrate.json', self.data)
        tools.save_croppings(pitch=self.pitch, data=self.data)

    def reshape(self):
        return np.array(self.data[self.drawing], np.int32).reshape((-1, 1, 2))

    def draw_poly(self, points):
        cv2.polylines(self.image, [points], True, self.color)
        cv2.imshow(FRAME_NAME, self.image)

    def get_zone(self, key, message):
        print '%s. %s' % (message, "Continue by pressing q")
        self.drawing, k = key, True

        while k != ord('q'):
            cv2.imshow(FRAME_NAME, self.image)
            k = cv2.waitKey(100) & 0xFF

        self.draw_poly(self.reshape())

    def get_pitch_outline(self):
        """
        Let user select points that corespond to the pitch outline.
        End selection by pressing 'q'.
        Result is masked and cropped.
        """
        self.get_zone('outline', 'Draw the outline of the pitch. Contine by pressing \'q\'')

        # Setup black mask to remove overflows
        self.image = tools.mask_pitch(self.image, self.data[self.drawing])

        # Get crop size based on points
        size = tools.find_crop_coordinates(self.image, self.data[self.drawing])
        # Crop
        self.image = self.image[size[2]:size[3], size[0]:size[1]]

        cv2.imshow(FRAME_NAME, self.image)

    def draw(self, event, x, y, flags, param):
        """
        Callback for events
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            color = self.color
            cv2.circle(self.image, (x - 1, y - 1), 2, color, -1)
            self.data[self.drawing].append((x, y))

    def get_goal(self, zone):
        """
        Returns the top and bottom corner of the goal in zone.
        """
        coords = self.data[zone]
        reverse = int(zone[-1]) % 2
        goal_coords = sorted(coords, reverse=reverse)[:2]
        if goal_coords[0][1] > goal_coords[1][1]:
            topCorner = goal_coords[1]
            bottomCorner = goal_coords[0]
        else:
            topCorner = goal_coords[0]
            bottomCorner = goal_coords[1]
        self.data[zone + '_goal'] = [topCorner, bottomCorner]

    def calibrateColor(self, cam):
        colours = OrderedDict([
            ("red",[]),
            ("yellow",[]),
            ("blue",[]),
            ("green",[]),
            ("pink",[])
        ])

        frames = []
        cnt = 2
        for c in xrange(cnt):
            frames.append(cam.get_frame())

        frame = np.mean(np.asarray(frames).astype(int), axis=0).astype(int)
        # todo: figure out why this does not work

        frame = frames[0]

        # frame =  cv2.GaussianBlur(frame, (9, 9), 0)


        cv2.imshow(self.config.COLCAL_TITLE, frame)
        normalizing_factors = np.sum(frame, axis=2)
        normalizing_factors[normalizing_factors == 0] = 1
        h, w = normalizing_factors.shape
        norm = frame.astype(float) / normalizing_factors.reshape(h, w, 1)
        norm *= 255
        norm = norm.astype(np.uint8)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for colour in colours:
            c=1
            print("click on {}, to continue press 'q'".format(colour))

            cv2.setMouseCallback(self.config.COLCAL_TITLE,
                                 lambda event, x, y, flags, param:self.p(event, x, y, flags, param, hsv, colours[colour], colour))
            while c!=113: # q
                if c==27:
                    cv2.destroyWindow(self.config.COLCAL_TITLE)
                    return None
                c = cv2.waitKey(100) & 0xFF
            else:
                l = colours[colour]
                if len(l) > 0:
                    min = np.min(np.asarray(l), axis=0)
                    max = np.max(np.asarray(l), axis=0)
                    colours[colour] = {
                        "min":min,
                        "max":max,
                    }
                else:
                    colours[colour] = None

        cv2.destroyWindow(self.config.COLCAL_TITLE)
        return colours

    def p(self, event, x, y, flags, param,frame, lst, colour_name):
        if event == cv2.EVENT_LBUTTONDOWN:
            windowSize = 1
            # print (frame[y][x])
            data = np.asarray(frame[y-windowSize:y+windowSize+1,x-windowSize:x+windowSize+1])
            height, width, d = data.shape
            data = data.reshape((width*height, 3))
            
            for pixel in data:
                if colour_name == 'red' and pixel[0] <= 16:
                    pixel[0] += 180
            	if (pixel[0] >= self.config.colours[colour_name]["min"][0] and
                    pixel[0] <= self.config.colours[colour_name]["max"][0]):
            	    lst.append(pixel)
            	    print "pixel recorded"
            	    print pixel
