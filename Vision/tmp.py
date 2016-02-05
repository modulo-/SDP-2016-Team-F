import cv2
import matplotlib
from vision.vision import Vision
from vision.camera import Camera
from vision.GUI import GUI
import vision.tools as tools
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
from vision.tracker import BallTracker, RobotTracker
from vision.findHSV import CalibrationGUI
from vision.tracker import Tracker
import time

import numpy as np

# C = cal.Configure(0)
# C.run(True)

class vision():
    def __init__(self):
        self.cam = Camera()
        self.calibrate()

    def calibrate(self):
        frame = self.cam.get_frame()

        """
        c = tools.get_colors(0)
        b = fh.CalibrateGUI(c)

        b.show(frame)
        """

        scalibration = tools.get_colors(0)
        #  print scalibration

        self.vision = Vision(
            pitch=0,
            color='blue',
            our_side='left',
            frame_shape=frame.shape,
            frame_center=self.cam.get_adjusted_center(frame),
            calibration=scalibration)

        # Set up postprocessing for vision

        # self.GUI = GUI(calibration=scalibration, pitch=0)

    def getPreprocessed(self):
        preprocessing = Preprocessing()

        postprocessing = Postprocessing()

        frame = self.cam.get_frame()
        pre_options = preprocessing.options
        # Apply preprocessing methods toggled in the UI
        preprocessed = preprocessing.run(frame, pre_options)

        height, width, channels = frame.shape

        model_positions, regular_positions = self.vision.locate(frame)
        model_positions = postprocessing.analyze(model_positions)

        #	print model_positions
        # frame_h = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # print frame[(464-128), 111]

        return preprocessed

    def showFrame(self):
        c = True

        counter = 1L
        timer = time.clock()
        while c != 27:
            preprocessed = self.getPreprocessed()

            # if 'background_sub' in preprocessed:
            #     cv2.imshow('bg sub', preprocessed['background_sub'])
            #     cv2.waitKey()

            green_mask, green_res = self.green(preprocessed['frame'])
            cv2.imshow('frame', preprocessed['frame'])
            # cv2.imshow('res_g', green_res)
            # cv2.imshow('mask_g', green_mask)

            c = cv2.waitKey(2) & 0xFF
            actions = []
            fps = float(counter) / (time.clock() - timer)
            """
            GUI.draw(
                frame, model_positions, actions, regular_positions, fps, None,
                None, None, None, False,
                our_color='blue', our_side = 'left', key=c, preprocess=pre_options)
            """
        else:
            cv2.destroyAllWindows()

    def green(self, frame):

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        lower_green = np.array([50, 100, 100])
        upper_green = np.array([70, 400, 400])

        # ([50, 100, 100])
        # 70, 255, 255]

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        return mask, res

if __name__=="__main__":
    v = vision()
    v.showFrame()
