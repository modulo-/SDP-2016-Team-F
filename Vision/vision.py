import math
import re
import time
from Queue import Queue
from functools import partial

import cv2
import numpy as np

import filters
from calibrate import Calibrate
from camera import Camera
from config import Config
from preprocessing import Preprocessing
from tracker import DotTracker
from world import World, Vector
from scipy.spatial import distance as sp_dist


class Vision:
    def __init__(self, video_port=0, pitch=None, planner_callback=None):
        self.config = Config()

        while pitch is None:
            req_room = raw_input("Enter Pitch Room [{opts}]: ".format(opts="/".join(self.config.pitch_room.options)))
            pitch_no = self.config.pitch_room.getCode(req_room, unifier=lambda str: re.sub(r'\W+', '', str.upper()))
            if pitch_no is not None:
                pitch = pitch_no
            else:
                print("Try again")

        self.cam = Camera(port=video_port, pitch=pitch, config=self.config)
        print("Camera initialised")
        if planner_callback is None:
            self.planner_callback = lambda: None
        else:
            self.planner_callback = planner_callback

        c = Calibrate(self.cam, self.config)
        self.vision = c.calibrateCam(self.cam, self.config)
        print("Basic camera calibration complete")
        # colours = c.calibrateColor(self.cam)

        # if colours is not None:
        #     for colour, data in colours.iteritems():
        #         if data is not None:
        #             for field in data:
        #                 self.config.colours[colour][field] = np.uint8([[data[field]]])
        #     print("Colors recorded")
        # else:
        #     print("Colors calibration skipped")

        self.config.addFilter("overlay", partial(filters.filter_overlay, self.world_latest, self.world_previous),
                              default=1)
        self.config.addFilter("grayscale", filters.filter_grayscale)
        self.config.addFilter("normalised", filters.filter_normalize)
        self.config.addFilter("red", partial(filters.filter_colour, "red"))
        self.config.addFilter("yellow", partial(filters.filter_colour, "yellow"))
        self.config.addFilter("blue", partial(filters.filter_colour, "blue"))
        self.config.addFilter("green", partial(filters.filter_colour, "green"))
        self.config.addFilter("pink", partial(filters.filter_colour, "pink"))
        self.config.addFilter("manual colour", partial(filters.filter_colour, None))
        print("Filters set up")

        print("Initialising trackers")

        self.tracker_ball = DotTracker(0, 0, 'red', self.config, "ball")
        self.tracker_blue = DotTracker(0, 0, 'yellow', self.config, "robot")
        self.tracker_yellow = DotTracker(0, 0, 'blue', self.config, "robot")

        self.world_latest = World()
        self.world_previous = None

        self.config.GUI()

        self.display()

    def getLatestWorld(self):
        return self.world_latest

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

            if self.world_latest is None:
                w = World()
            else:
                self.world_previous = self.world_latest
                w = self.world_latest
                w.time = time.time()

            preprocessed = self.getPreprocessed()
            frame = preprocessed['frame']

            q = Queue()
            while not q.empty():  # make sure no old stuff is in the queue, like when overlay wasn't drawn
                print q.get()

            h, w, d = frame.shape

            # note hte potential to detect in threads and then join back!
            self.tracker_ball.find(frame, q)
            self.tracker_blue.find(frame, q)
            self.tracker_yellow.find(frame, q)

            while not q.empty():
                item = q.get_nowait()
                if item is None:
                    continue
                x, y = item['x'], item['y']

                if item['name'] == "ball":
                    if self.world_previous is not None and self.world_previous.ball is not None:
                        angle = math.atan2(self.world_previous.ball.y - y, self.world_previous.ball.x - x)
                        distance = sp_dist.euclidean((x, y), (self.world_previous.ball.x, self.world_previous.ball.y))
                        t = w.time - self.world_previous.time
                        velocity = distance / t
                    else:
                        angle = 0
                        velocity = 0
                    w.ball = Vector(x, y, angle, velocity)
                elif "robot" in item['name']:
                    if item['colour'] == 'blue':
                        if item['identification'] == 'green':
                            w.robot_blue_green = Vector(x, y, math.radians(item['orientation']), 0)
                        elif item['identification'] == 'pink':
                            w.robot_blue_pink = Vector(x, y, math.radians(item['orientation']), 0)
                    if item['colour'] == 'blue':
                        if item['identification'] == 'green':
                            w.robot_yellow_green = Vector(x, y, math.radians(item['orientation']), 0)
                        elif item['identification'] == 'pink':
                            w.robot_yellow_pink = Vector(x, y, math.radians(item['orientation']), 0)

            self.world_latest = w
            self.planner_callback(self.world_latest)

            # found items will be stored in the queue, and accessed if/when drawing the overlay

            for name in self.config.filter_stack:
                filter = self.config.filters[name]
                if filter["option"].selected:
                    frame = filter["function"](frame)
                    cv2.imshow(name, frame)
                else:
                    print("Filter stack does not agree with activated filters")

            cv2.imshow(self.config.OUTPUT_TITLE, frame)
            cv2.setMouseCallback(self.config.OUTPUT_TITLE,
                                 lambda event, x, y, flags, param: self.p(event, x, y, flags, param, frame))

            c = cv2.waitKey(50) & 0xFF
        else:
            cv2.destroyAllWindows()

    def p(self, event, x, y, flags, param, frame):

        if event == cv2.EVENT_LBUTTONDOWN:
            print y, x
            print frame[y][x]


if __name__ == "__main__":
    Vision(pitch=0)
