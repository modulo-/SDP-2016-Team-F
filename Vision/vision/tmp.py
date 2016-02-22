import cv2
import calibrate as cal
import matplotlib
import camera
import findHSV as fh
import tools
import vision
import GUI as g

# C = cal.Configure(0)
# C.run(True)

cam = camera.Camera()
frame = cam.get_frame()
cv2.imshow('frame', frame)
cv2.waitKey()

"""
c = tools.get_colors(0)
b = fh.CalibrateGUI(c)

b.show(frame)
"""
"""
scalibration = tools.get_colors(0)
#  print scalibration

v = vision.Vision(
    pitch=0,
    color='blue',
    our_side='left',
    frame_shape=frame.shape,
    frame_center=cam.get_adjusted_center(frame),
    calibration=scalibration)

# Set up postprocessing for vision
# postprocessing = Postprocessing()

GUI = g.GUI(calibration=scalibration, pitch=0)


# preprocessing = Preprocessing()
        """
