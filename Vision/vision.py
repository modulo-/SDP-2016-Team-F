from Queue import Queue
from functools import partial
import cv2
import re
from camera import Camera
from preprocessing import Preprocessing
from tracker import DotTracker
from calibrate import Calibrate
import filters
from config import Config


class Vision:
    def __init__(self, video_port=0, pitch=None):
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

        self.detected_objects = Queue()
        self.config.addFilter("overlay", partial(filters.filter_overlay, self.detected_objects), default=1)
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
        self.tracker_us = DotTracker(0, 0, 'yellow', self.config, "robot")
        self.tracker_them = DotTracker(0, 0, 'blue', self.config, "robot")

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
            while not q.empty():  # make sure no old stuff is in the queue, like when overlay wasn't drawn
                print q.get()

            h, w, d = frame.shape

            # note hte potential to detect in threads and then join back!
            self.tracker_ball.find(frame, q)
            self.tracker_us.find(frame, q)
            self.tracker_them.find(frame, q)

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
