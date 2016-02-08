import math


class CommsManager:

    def __init__(self, robot):
        self.robot = robot

    def move(self, distance):
        print("Moving robot {0} distance {1}".format(self.robot, distance))

    def move_to(self, x, y):
        heading = math.degrees(math.atan2(y, x))
        distance = math.sqrt(x**2 + y**2)
        print("Turning robot {0} to head {1} and moving distance {2}".format(self.robot, heading, distance))

    def turn(self, angle):
        print("Turning robot {0} angle {1}".format(self.robot, angle))

    def kick(self, distance):
        print("Robot {0} kicking distance {1}".format(self.robot, distance))

    def kick_full_power(self):
        print("Robot {0} kicking full power".format(self.robot))

    def close_grabbers(self):
        print("Robot {0} closing grabbers".format(self.robot))

    def release_grabbers(self):
        print("Robot {0} releasing grabbers".format(self.robot))
