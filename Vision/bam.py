import cv2

from oldvision.vision import _Vision
from camera import Camera
from oldvision.GUI import GUI
import oldvision.tools as tools
from postprocessing.postprocessing import Postprocessing
from preprocessing.preprocessing import Preprocessing
from logging import debug


# C = cal.Configure(0)
# C.run(True)

cam = Camera(pitch=0)
frame = cam.get_frame()

"""
c = tools.get_colors(0)
b = fh.CalibrateGUI(c)

b.show(frame)
"""

scalibration = tools.get_colors(0)
#  print scalibration

vision = _Vision(
pitch=0,
color='blue',
our_side='left',
frame_shape=frame.shape,
frame_center=cam.get_adjusted_center(frame),
calibration=scalibration)

# Set up postprocessing for oldvision

postprocessing = Postprocessing()

GUI = GUI(calibration=scalibration, pitch=0)


preprocessing = Preprocessing()

frame = cam.get_frame()
pre_options = preprocessing.options
# Apply preprocessing methods toggled in the UI
preprocessed = preprocessing.run(frame, pre_options)
frame = preprocessed['frame']
if 'background_sub' in preprocessed:
	cv2.imshow('bg sub', preprocessed['background_sub'])
	cv2.waitKey()

height, width, channels = frame.shape

# frame = frame[425:445, 5:25]

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

debug(frame_hsv)
debug(frame)

cv2.imshow('frame', frame)
cv2.waitKey()
cv2.destroyAllWindows()

