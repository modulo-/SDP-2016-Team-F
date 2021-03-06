import cv2
import filters
import oldvision.tools as tools
import numpy as np


class Camera(object):
    """
    Camera access wrapper.
    """

    def __init__(self, pitch, port=0, config=None):
        self.config = config
        self.capture = cv2.VideoCapture(port)
        calibration = tools.get_croppings(pitch=pitch)
        self.crop_values = tools.find_extremes(calibration['outline'])

        if pitch == 0:
        	self.capture.set(cv2.CAP_PROP_BRIGHTNESS, 0.39)
        	self.capture.set(cv2.CAP_PROP_CONTRAST, 0.5)
        	self.capture.set(cv2.CAP_PROP_SATURATION, 0.39)
        	self.capture.set(cv2.CAP_PROP_HUE, 0.5)
       	else:
	    	self.capture.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
        	self.capture.set(cv2.CAP_PROP_CONTRAST, 0.5)
        	self.capture.set(cv2.CAP_PROP_SATURATION, 0.39)
        	self.capture.set(cv2.CAP_PROP_HUE, 0.5)

        # Parameters used to fix radial distortion
        radial_data = tools.get_radial_data()
        self.nc_matrix = radial_data['new_camera_matrix']
        self.c_matrix = radial_data['camera_matrix']
        self.dist = radial_data['dist']

    def get_frame(self, radial_dist=0):
        """
        Retrieve a frame from the camera.

        Returns the frame if available, otherwise returns None.
        """
        # status, frame = True, cv2.imread('img/i_all/00000003.jpg')

        status, frame = self.capture.read()
        frame = self.fix_radial_distortion(frame)
        # frame = cv2.flip(frame, 0)


        frame = cv2.GaussianBlur(frame, (7, 7), 0)
        # print type(frame), type(frame[0][0][0])
        # frame = filters.filter_normalize(frame, self.config)

        if status:
            return frame[
                self.crop_values[2]:self.crop_values[3],
                self.crop_values[0]:self.crop_values[1]
            ]

    def fix_radial_distortion(self, frame):
        return cv2.undistort(
            frame, self.c_matrix, self.dist, None, self.nc_matrix)

    def get_adjusted_center(self, frame):
        return (320-self.crop_values[0], 240-self.crop_values[2])
