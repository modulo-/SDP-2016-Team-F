import cv2
import numpy as np
import matplotlib.pyplot as plt

CONTROL = ["Lower threshold for hue",
           "Upper threshold for hue",
           "Lower threshold for saturation",
           "Upper threshold for saturation",
           "Lower threshold for value",
           "Upper threshold for value",
           "Contrast", 
           "Gaussian blur",
           "Open",
           "Dilation",
           "Erode"]

MAXBAR = {"Lower threshold for hue":360,
          "Upper threshold for hue":360,
          "Lower threshold for saturation":255,
          "Upper threshold for saturation":255,
          "Lower threshold for value":255,
          "Upper threshold for value":255,
          "Contrast":100,
          "Gaussian blur":100,
          "Open": 100,
          "Dilation": 100,
          "Erode":100
        }

INDEX = {"Lower threshold for hue":0,
         "Upper threshold for hue":0,
         "Lower threshold for saturation":1,
         "Upper threshold for saturation":1,
         "Lower threshold for value":2,
         "Upper threshold for value":2
        }

KEYS = {ord('y'):'yellow',
        ord('r'):'red',
        ord('b'):'blue',
        ord('d'):'dot',
        ord('p'):'plate'}

def nothing(x):
    pass

class CalibrationGUI(object):
    """
    This class caters for the creation of
    the hue, saturation, value, contrast and
    blur threshold trackbars
    """
    def __init__(self, calibration):
        self.color = 'plate'
        # self.pre_options = pre_options
        self.calibration = calibration
        self.maskWindowName = "Mask " + self.color

        self.setWindow()

    def setWindow(self):

        cv2.namedWindow(self.maskWindowName)
        # print self.calibration
        createTrackbar = lambda setting, \
                                value: \
                                    cv2.createTrackbar(
                                        setting,
                                        self.maskWindowName,
                                        int(value),
                                        MAXBAR[setting], nothing)
        createTrackbar('Lower threshold for hue',
                       self.calibration[self.color]['min'][0])
        createTrackbar('Upper threshold for hue',
                       self.calibration[self.color]['max'][0])
        createTrackbar('Lower threshold for saturation',
                       self.calibration[self.color]['min'][1])
        createTrackbar('Upper threshold for saturation',
                       self.calibration[self.color]['max'][1])
        createTrackbar('Lower threshold for value',
                       self.calibration[self.color]['min'][2])
        createTrackbar('Upper threshold for value',
                       self.calibration[self.color]['max'][2])
        createTrackbar('Contrast',
                       self.calibration[self.color]['contrast'])
        createTrackbar('Gaussian blur',
                       self.calibration[self.color]['blur'])
        createTrackbar('Open',
                       self.calibration[self.color]['open'])
        createTrackbar('Dilation',
                       self.calibration[self.color]['close'])
        createTrackbar('Erode',
                       self.calibration[self.color]['erode'])

    def change_color(self, color):
        """
        Changes the color mask within the GUI
        """
        cv2.destroyWindow(self.maskWindowName)
        self.color = color
        self.maskWindowName = "Mask " + self.color
        self.setWindow()

    def show(self, frame, key=None):

        if key != 255:
            try:
                self.change_color(KEYS[key])
            except:
                pass

        getTrackbarPos = lambda setting: cv2.getTrackbarPos(setting, self.maskWindowName)

        values = {}
        for setting in CONTROL:
            values[setting] = float(getTrackbarPos(setting))
        values['Gaussian blur'] = int(values['Gaussian blur'])

        self.calibration[self.color]['min'] = np.array(
                                                [values['Lower threshold for hue'],
                                                 values['Lower threshold for saturation'],
                                                 values['Lower threshold for value']])
        self.calibration[self.color]['max'] = np.array(
                                                    [values['Upper threshold for hue'],
                                                     values['Upper threshold for saturation'],
                                                     values['Upper threshold for value']])
        self.calibration[self.color]['contrast'] = values['Contrast']
        self.calibration[self.color]['blur'] = values['Gaussian blur']
        self.calibration[self.color]['open'] = values['Open']
        self.calibration[self.color]['close'] = values['Dilation']
        self.calibration[self.color]['erode'] = values['Erode']

        mask = self.get_mask(frame)
        cv2.imshow(self.maskWindowName, mask)

    # Duplicated from tracker.py
    def get_mask(self, frame):
        """
        NOTE THIS IS ONLY USED FOR DISLPAY PURPOSES
        GaussianBlur blur:
            G =     [[G11, ..., G1N],
                1/L      ...,
                     [GN1, ..., GNN]]
            G is the bluring kernel
            L = sqrt(dot(blur, blur))
            GII Gaussian number

        params:
            frame: 
                description: camera image
                type: numpy array

        output:
            frame_mask;
                description: filtered (blured) camera image.
                    it is blured by the GUI given parameters.
                type: numpy array

        """
        # plt.imshow(frame)
        # plt.show()
        blur = self.calibration[self.color]['blur']
        if blur >= 1:
            if blur % 2 == 0:
                blur += 1
            frame = cv2.GaussianBlur(frame, (blur, blur), 0)

        contrast = self.calibration[self.color]['contrast']
        if contrast >= 1.0:
            frame = cv2.add(frame, np.array([contrast]))

        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        min_color = self.calibration[self.color]['min']
        max_color = self.calibration[self.color]['max']
        frame_mask = cv2.inRange(frame_hsv, min_color, max_color)
        if self.calibration[self.color]['open'] >= 1:
                kernel = np.ones((2,2 ),np.uint8)
                frame_mask = cv2.morphologyEx(frame_mask,
                                              cv2.MORPH_OPEN,
                                              kernel,
                                              iterations=self.calibration[self.color]['open'])
        if self.calibration[self.color]['close'] >= 1:
                kernel = np.ones((2,2 ),np.uint8)
                frame_mask = cv2.dilate(frame_mask,
                                        kernel,
                                        iterations=self.calibration[self.color]['close'])
        if self.calibration[self.color]['erode'] >= 1:
                kernel = np.ones((2,2 ),np.uint8)
                frame_mask = cv2.erode(frame_mask,
                                        kernel,
                                        iterations=self.calibration[self.color]['erode'])


        return frame_mask
