from collections import OrderedDict
import cv2
import numpy as np


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
    colour = Option("colour", text="Our colour", options=["yellow", "blue"])
    side = Option("side", text="Our side", options=["left", "right"])

    colours = {
        "red": {
            "max": np.array([122, 133, 255]),
            "mean": np.array([108.66666667, 109.25, 230.75]),
            "min": np.array([79, 89, 151])
        },
        "yellow": {
            'max': np.array([186, 255, 255]),
            'mean': np.array([180.4, 255., 255.]),
            'min': np.array([175, 255, 255])
        },
        "blue": {
            'max': np.array([239, 216, 212]),
            'mean': np.array([229.71428571, 206., 170.]),
            'min': np.array([219, 196, 145])
        },
        "green": {
            'max': np.array([188, 255, 219]),
            'mean': np.array([159., 254.75, 201.5]),
            'min': np.array([139, 251, 191])
        },
        "pink": {
            'max': np.array([216, 168, 255]),
            'mean': np.array([207.13333333, 153.86666667, 255.]),
            'min': np.array([194, 145, 255])
        }
    }

    dot_areas = {
        'blue': 2,
        'yellow': 5,
        'red': 15
    }
    filters = OrderedDict()
    filter_stack = []  # OrderedSet()
    OUTPUT_TITLE = 'Filter Output'
    FILTER_TITLE = 'Vision Parameters'
    COLCAL_TITLE = 'Colour Calibration'

    def __init__(self):
        pass

    def GUI(self):
        cv2.namedWindow(self.FILTER_TITLE)

        self.createTrackbar(self.colour)
        self.createTrackbar(self.side)

        for name, filter in self.filters.iteritems():
            self.createTrackbar(filter["option"])

    def createTrackbar(self, option):
        cv2.createTrackbar(option.text, self.FILTER_TITLE, option.selected, len(option.options) - 1,
                           lambda x: self.toggle(option, x))

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
