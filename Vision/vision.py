import math
import re
import time
from Queue import Queue
from functools import partial
from logging import info

import cv2
import numpy as np
import copy
from numpy import pi

import filters
from calibrate import Calibrate
from camera import Camera
from config import Config
from preprocessing import Preprocessing
from calibrations import colour_profiles
from tracker import DotTracker
from world import World, Vector
from scipy.spatial import distance as sp_dist


class Vision:
    def __init__(self, video_port=0, pitch=None, planner_callback=None):
        self.config = Config(self)
        while pitch is None:
            req_room = raw_input("Enter Pitch Room [{opts}]: ".format(opts="/".join(self.config.pitch_room.options)))
            pitch_no = self.config.pitch_room.getCode(req_room, unifier=lambda str: re.sub(r'\W+', '', str.upper()))
            if pitch_no is not None:
                pitch = pitch_no

        
        self.config.pitch_room.selected = pitch
        if pitch == 0:
            self.config.colours = colour_profiles['pitch_3d03']
        elif pitch == 1:
            self.config.colours = colour_profiles['pitch_3d04']

        self.cam = Camera(port=video_port, pitch=pitch, config=self.config)
        #print("Camera initialised")

        def print_function(x):
            info(x)

        if planner_callback is None:
            self.planner_callback = (lambda x: print_function(x))
        else:
            self.planner_callback = planner_callback

        c = Calibrate(self.cam, self.config)

        self.world_latest = World()
        self.world_previous = None
        
        # c.run(True)
        
        print("Basic camera calibration complete")
        colours = c.calibrateColor(self.cam)
        all_colors = np.empty([10, 3], dtype = np.uint8)
        color_id = 0

        if colours is not None:
            for colour, data in colours.iteritems():
                if data is not None:
                    for field in data:
                        self.config.colours[colour][field] = np.uint8(data[field])
                        all_colors[color_id] = np.uint8(data[field])
                        color_id += 1
            print("Colors recorded")
            np.save("color_calibrations", all_colors)
        else:
            print("Colors calibration skipped")
            all_colors = np.load("color_calibrations.npy")
            all_color_names = ['red', 'yellow', 'blue', 'green', 'pink']
            for i in range(0,5):
            	self.config.colours[all_color_names[i]]['max'] = all_colors[i*2]
            	self.config.colours[all_color_names[i]]['min'] = all_colors[i*2+1]

        self.config.addFilter("overlay", filters.filter_overlay,
                              default=1)
        self.config.addFilter("grayscale", filters.filter_grayscale)
        self.config.addFilter("normalised", filters.filter_normalize)
        self.config.addFilter("red", partial(filters.filter_colour, "red"))
        self.config.addFilter("yellow", partial(filters.filter_colour, "yellow"))
        self.config.addFilter("blue", partial(filters.filter_colour, "blue"))
        self.config.addFilter("green", partial(filters.filter_colour, "green"))
        self.config.addFilter("pink", partial(filters.filter_colour, "pink"))
        self.config.addFilter("manual colour", partial(filters.filter_colour, None))
        #print("Filters set up")

        #print("Initialising trackers")

        self.tracker_ball = DotTracker(0, 0, 'red', self.config, "ball")
        self.tracker_blue = DotTracker(0, 0, 'yellow', self.config, "robot")
        self.tracker_yellow = DotTracker(0, 0, 'blue', self.config, "robot")


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
                w = World(self.world_latest)

            preprocessed = self.getPreprocessed()
            frame = preprocessed['frame']
            
            height, width, dim = frame.shape
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            q = Queue()

            # note hte potential to detect in threads and then join back!
            #print("track ball")
            self.tracker_ball.find(frame_hsv, q)
            #print("track blue")
            self.tracker_blue.find(frame_hsv, q)
            #print("track yellow")
            self.tracker_yellow.find(frame_hsv, q)

            while not q.empty():
                item = q.get_nowait()
                #print(item)
                if item is None:
                    continue
                x, y = item['x'], item['y']

                if item['name'] == "ball":
                    if self.world_previous is not None and self.world_previous.ball is not None:
                        angle = math.atan2(y-self.world_previous.ball.y, x - self.world_previous.ball.x)
                        distance = sp_dist.euclidean((x, y), (self.world_previous.ball.x, self.world_previous.ball.y))
                        t = w.time - self.world_previous.time
                        # print time.time(), w.time, self.world_previous.time
                        velocity = distance / t /10
                    else:
                        angle = 0
                        velocity = 0

                    while angle > 2* math.pi:
                        angle-=math.pi
                    while angle < 0:
                        angle+=math.pi

                    w.ball = Vector(x, y, angle, velocity)
                elif "robot" in item['name']:
                    if item['orientation'] is None:
                        rad = 0
                        dis = 0
                    else:
                        rad = math.radians(item["orientation"]%360)
                        dis = 1

                    if item['colour'] == 'blue':
                        if item['identification'] == 'green':
                            w.robot_blue_green = Vector(x, y, rad, dis)
                        elif item['identification'] == 'pink':
                            w.robot_blue_pink = Vector(x, y, rad, dis)
                    if item['colour'] == 'yellow':
                        if item['identification'] == 'green':
                            w.robot_yellow_green = Vector(x, y, rad, dis)
                        elif item['identification'] == 'pink':
                            w.robot_yellow_pink = Vector(x, y, rad, dis)
            
            # quit()
            self.world_latest = w

            # found items will be stored in the queue, and accessed if/when drawing the overlay
            for name in self.config.filter_stack:
                filter = self.config.filters[name]
                if filter["option"].selected:
                    frame = filter["function"](frame)
                    cv2.imshow(name, frame)
                #else:
                    #print("Filter stack does not agree with activated filters")

            cv2.imshow(self.config.OUTPUT_TITLE, frame)
            cv2.setMouseCallback(self.config.OUTPUT_TITLE,
                                 lambda event, x, y, flags, param: self.p(event, x, y, flags, param, frame))

            c = cv2.waitKey(50) & 0xFF
            
            planner_w = copy.deepcopy(w);
            if planner_w.ball != None:
            	planner_w.ball.y = height - planner_w.ball.y
            if planner_w.robot_yellow_green != None:
            	planner_w.robot_yellow_green.y = height - planner_w.robot_yellow_green.y
            	planner_w.robot_yellow_green.angle = (pi/2 - planner_w.robot_yellow_green.angle)%(2*pi)
            if planner_w.robot_yellow_pink != None:
            	planner_w.robot_yellow_pink.y = height - planner_w.robot_yellow_pink.y
            	planner_w.robot_yellow_pink.angle = (pi/2 - planner_w.robot_yellow_pink.angle)%(2*pi)
            if planner_w.robot_blue_green != None:
            	planner_w.robot_blue_green.y = height - planner_w.robot_blue_green.y
            	planner_w.robot_blue_green.angle = (pi/2 - planner_w.robot_blue_green.angle)%(2*pi)
            if planner_w.robot_blue_pink != None:
            	planner_w.robot_blue_pink.y = height - planner_w.robot_blue_pink.y
            	planner_w.robot_blue_pink.angle = (pi/2 - planner_w.robot_blue_pink.angle)%(2*pi)
            	
            # print(planner_w.__dict__)
            
            self.planner_callback(planner_w)
        else:
            cv2.destroyAllWindows()

    def p(self, event, x, y, flags, param, frame):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            print y, x
            print frame[y][x]


if __name__ == "__main__":
    Vision()
