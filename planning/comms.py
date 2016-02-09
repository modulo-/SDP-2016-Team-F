import math

import group11cmd

class CommsManager:

    def __init__(self, robot):
        self.robot = robot

    def move(self, distance):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_strait(distance))
        print("Moving robot {0} distance {1}".format(self.robot, distance))

    def move_to(self, x, y):
        # Needs actual positions, and current positions of the robot!
        #if robot == 11:
        #    group11cmd.run(group11cmd.cmd_mv(x0, y0, angle0, x1, y1, angle1))
        heading = math.degrees(math.atan2(y, x))
        distance = math.sqrt(x**2 + y**2)
        print("Turning robot {0} to head {1} and moving distance {2}".format(self.robot, heading, distance))

    def turn(self, angle):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_spin(angle))
        print("Turning robot {0} angle {1}".format(self.robot, angle))

    def kick(self, distance):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_kick(300))
        print("Robot {0} kicking distance {1}".format(self.robot, distance))

    def kick_full_power(self):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_kick(300))
        print("Robot {0} kicking full power".format(self.robot))

    def close_grabbers(self):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_grabber_close())
        print("Robot {0} closing grabbers".format(self.robot))

    def release_grabbers(self):
        if robot == 11:
            group11cmd.run(group11cmd.cmd_grabber_open())
        print("Robot {0} releasing grabbers".format(self.robot))
