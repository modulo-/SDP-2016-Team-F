from Queue import Queue
from functools import partial
import cv2
import re
from oldvision.vision import _Vision
from camera import Camera
import oldvision.tools as tools
from preprocessing import Preprocessing
from postprocessing import Postprocessing
from tracker import BallTracker, RobotTracker
# # from oldvision.findHSV import CalibrationGUI
#from tracker import Tracker
from calibrate import Calibrate
import time
import numpy as np
import filters

from config import Config


class Vision:
    def __init__(self, video_port=0, pitch=None):
        self.config = Config()

        while pitch is None:
            req_room = raw_input("Enter Pitch Room [{opts}]: ".format(opts = "/".join(self.config.pitch_room.options)))
            pitch_no = self.config.pitch_room.getCode(req_room, unifier = lambda str:re.sub(r'\W+', '', str.upper()))
            if pitch_no is not None:
                pitch = pitch_no
            else:
                print("Try again")

        self.cam = Camera(port=video_port, pitch=pitch)
        print("Camera initialised")

        c = Calibrate(self.cam, self.config)
        self.vision = c.calibrateCam(self.cam, self.config)
        print("Basic camera calibration complete")
        colours = c.calibrateColor(self.cam)

        if colours is not None:
            for colour, data in colours.iteritems():
                if data is not None:
                    self.config.colours[colour] = np.uint8([[data]])
            print("Colors recorded")
        print("Colors calibration skipped")

        self.detected_objects = Queue()
        self.config.addFilter("overlay", partial(filters.filter_overlay, self.detected_objects), default=1)
        self.config.addFilter("grayscale", filters.filter_grayscale)
        self.config.addFilter("normalised", filters.filter_normalize)
        self.config.addFilter("red", partial(filters.filter_colour, "red"))
        self.config.addFilter("yellow", partial(filters.filter_colour, "yellow"))
        self.config.addFilter("blue", partial(filters.filter_colour, "blue"))
        self.config.addFilter("green", partial(filters.filter_colour, "green"))
        self.config.addFilter("pink", partial(filters.filter_colour, "pink"))
        print("Filters set up")
        self.config.GUI()

        self.display()



    def getPreprocessed(self):
        preprocessing = Preprocessing()
        # postprocessing = Postprocessing()

        frame = self.cam.get_frame()
        pre_options = preprocessing.options
        # Apply preprocessing methods toggled in the UI
        preprocessed = preprocessing.run(frame, pre_options)

        height, width, channels = frame.shape

        # model_positions, regular_positions = self.vision.locate(frame)
        # model_positions = postprocessing.analyze(model_positions)

        #	print model_positions
        # frame_h = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # print frame[(464-128), 111]

        return preprocessed

    def display(self):
        c = True
        while c != 27:
            # self.config.GUI()

            preprocessed = self.getPreprocessed()
            frame = preprocessed['frame']

            q = self.detected_objects
            while not q.empty(): # make sure no old stuff is in the queue, like when overlay wasn't drawn
                print q.get()

            h, w, d = frame.shape
            col = self.config.colours

            #note hte potential to detect in threads and then join back!
            r=BallTracker((0,w,0,h),0,0,col['red'], "red-ball")
            r.find(frame, q)
            r=BallTracker((0,w,0,h),0,0,col['yellow'], "yellow-dot")
            r.find(frame, q)
            r=BallTracker((0,w,0,h),0,0,col['blue'], "blue-dot")
            r.find(frame, q)

            # found items will be stored in the queue, and accessed if/when drawing the overlay

            for name in self.config.filter_stack:
                filter = self.config.filters[name]
                if filter["option"].selected:
                    frame = filter["function"](frame)
                else:
                    print("Filter stack does not agree with activated filters")

            cv2.imshow(self.config.OUTPUT_TITLE, frame)
            cv2.setMouseCallback(self.config.OUTPUT_TITLE,
                             lambda event, x, y, flags, param:self.p(event, x, y, flags, param, frame))


            c = cv2.waitKey(5000) & 0xFF
        else:
            cv2.destroyAllWindows()

    def p(self, event, x, y, flags, param,frame):

        if event == cv2.EVENT_LBUTTONDOWN:
            print y, x
            print frame[y][x]

if __name__ == "__main__":
    Vision(pitch=0)
