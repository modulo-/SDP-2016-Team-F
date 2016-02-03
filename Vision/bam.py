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

# C = cal.Configure(0)
# C.run(True)

cam = Camera()
frame = cam.get_frame()

"""
c = tools.get_colors(0)
b = fh.CalibrateGUI(c)

b.show(frame)
"""

scalibration = tools.get_colors(0)
#  print scalibration

vision = Vision(
pitch=0,
color='blue',
our_side='left',
frame_shape=frame.shape,
frame_center=cam.get_adjusted_center(frame),
calibration=scalibration)

# Set up postprocessing for vision

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

frame = frame[425:445, 5:25]

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

print frame_hsv
print frame

cv2.imshow('frame', frame)
cv2.waitKey()
cv2.destroyAllWindows()

