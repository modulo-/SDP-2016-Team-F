from __future__ import division
import math
from math import pi

class CommsManager(object):

    def __init__(self, robot):
        self.robot_index = robot

    def move(self, distance):
        print("Moving robot {0} distance {1}".format(self.robot_index, distance))

    def turn(self, angle):
        print("Turning robot {0} angle {1} radians ( {2} degrees)".format(self.robot_index, angle, math.degrees(angle)))

    def kick(self, distance):
        print("Robot {0} kicking distance {1}".format(self.robot_index, distance))

    def kick_full_power(self):
        print("Robot {0} kicking full power".format(self.robot_index))

    def close_grabbers(self):
        print("Robot {0} closing grabbers".format(self.robot_index))

    def release_grabbers(self):
        print("Robot {0} releasing grabbers".format(self.robot_index))
