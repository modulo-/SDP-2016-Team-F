from collections import OrderedDict
import cv2
import numpy as np

from Vision import calibrations
from Vision.calibrations import colour_profiles


class Option(object):
    @property
    def text(self):
        return self._text + " ({opts})".format(opts="/".join(self.options))

    @property
    def filter(self):
        return self.type == "filter"

    @property
    def options(self):
        return self._options

    @property
    def selected(self):
        return self._selected

    @selected.setter
    def selected(self, value):
        self._selected = value

    @property
    def selected_option(self):
        return self.options[self._selected]

    def __init__(self, name, text="Description missing", options=list(), default=0, type="general"):
        self._text = str(text)
        self.type = type
        self.name = name

        if not isinstance(options, list):
            raise Exception("Options should be passed as a list")
        if len(options) == 0:
            raise Exception("Options list empty")
        else:
            self._options = options

        self._selected = default

    def getCode(self, value, unifier=str):
        try:
            return [unifier(o) for o in self.options].index(unifier(value))
        except ValueError:
            return None


class Config:
    pitch_room = Option("pitch_room", text="Pitch room", options=["3.D03", "3.D04"])
    computer = Option("colour", text="Our colour", options=list(calibrations.colour_profiles))

    dot_areas = {
        'blue': 10,
        'yellow': 10,
        'red': 20
    }
    
    filters = OrderedDict()
    filter_stack = []  # OrderedSet()
    OUTPUT_TITLE = 'Filter Output'
    FILTER_SELECTION = 'Filter Selection'
    FILTER_PARAMS = 'Vision Parameters'
    COLCAL_TITLE = 'Colour Calibration'

    low = [0,0,0]
    high = [0,0,0]
    open =-1
    close =-1
    erode =-1
    dilate =-1

    delta_angle = 23

    def __init__(self):
        cv2.namedWindow(self.FILTER_SELECTION)
        cv2.namedWindow(self.FILTER_PARAMS)

    def GUI(self):
        self.createTrackbar(self.computer, callback=lambda x: self.set(self.computer.selected, x,
              lambda : self.set("colours", colour_profiles[self.computer.selected_option])))

        for name, filter in self.filters.iteritems():
            self.createTrackbar(filter["option"])

        cv2.createTrackbar("H_low", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("low", 0, x))
        cv2.createTrackbar("S_low", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("low", 1, x))
        cv2.createTrackbar("V_low", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("low", 2, x))
        
        cv2.createTrackbar("H_high", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("high", 0, x))
        cv2.createTrackbar("S_high", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("high", 1, x))
        cv2.createTrackbar("V_high", self.FILTER_PARAMS, 0, 255,
                           lambda x: self.setCol("high", 2, x))


        cv2.createTrackbar("delta_angle", self.FILTER_PARAMS, self.delta_angle, 360,
                           lambda x: self.set("delta_angle", x))

        cv2.createTrackbar("open", self.FILTER_PARAMS, 0, 10,
                           lambda x: self.set("open", x*2-1))
        cv2.createTrackbar("close", self.FILTER_PARAMS, 0, 10,
                           lambda x: self.set("close", x*2-1))
        cv2.createTrackbar("erode", self.FILTER_PARAMS, 0, 10,
                           lambda x: self.set("erode", x*2-1))
        cv2.createTrackbar("dilate", self.FILTER_PARAMS, 0, 10,
                           lambda x: self.set("dilate", x*2-1))
        
    def setCol(self, bound, col, val):
        c = getattr(self, bound)
        c[col]=val

    def set(self, bound, val, callback = None):
        c = getattr(self, bound)
        c.selected=val
        if callback is not None:
            callback()


    def createTrackbar(self, option, callback = None):
        if callback is None:
            callback = lambda x: self.toggle(option, x)

        cv2.createTrackbar(option.text, self.FILTER_SELECTION, option.selected, len(option.options) - 1,callback)

    def addFilter(self, name, func, default=0):
        if default not in [0, 1]:
            return 1 if str(default) == "yes" else 0

        opt = Option(name, text="Display {}".format(name), options=["no", "yes"], default=default, type="filter")
        self.filters[name] = {
            "option": opt,
            "function": lambda frame: func(frame, self)
        }

        if default == 1:
            self.filter_stack.append(name)

    def toggle(self, option, x):

        option.selected = x
        if option.filter:
            if x == 1:
                try:
                    self.filter_stack.remove("overlay")
                    overlay = True
                except ValueError:
                    overlay = False

                if option.name in self.filter_stack:
                    self.filter_stack.remove(option.name)
                self.filter_stack.append(option.name)
                if overlay:
                    self.filter_stack.append("overlay")
            else:
                try:
                    self.filter_stack.remove(option.name)
                except ValueError:
                    pass
            print "Filter stack", self.filter_stack
