import math
import rf_comms as comms


class CommsManager(object):

    def __init__(self, robot):
        self.robot_index = robot

    def move(self, distance):
        print("Robot {0} moving distance {1}".format(self.robot_index, distance))

    def turn(self, angle):
        print("Robot {0} turning by angle {1}".format(self.robot_index, angle))

    def kick(self, distance):
        print("Robot {0} kicking distance {1}".format(self.robot_index, distance))

    def kick_full_power(self):
        print("Robot {0} kicking full power".format(self.robot_index))

    def close_grabbers(self):
        print("Robot {0} closing grabbers".format(self.robot_index))

    def release_grabbers(self):
        print("Robot {0} releasing grabbers".format(self.robot_index))

class RFCommsManager (CommsManager):

    def __init__(self, robot, serial_device):
        # group 11 - 1, group 12 - 2
        robot_id = 2
        # hex
        rf_channel = "67"
        guard_chars = "~~~"
        chunk_size = 30
        comms.init(serial_device, rf_channel, guard_chars, listen=True)
        super(RFCommsManager, self).__init__(robot)

    # move a distance in mm
    def move(self, distance):
        cmd = b"m"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
        super(RFCommsManager, self).move(distance)
    
    # turn by an angle in degrees
    def turn(self, angle):
        cmd = b"t"+struct.pack(">h", angle)
        comms.send(cmd, robot_id)
        super(RFCommsManager, self).turn(angle)
    
    # kick a distance in cm
    def kick(self, distance):
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
        super(RFCommsManager, self).kick(distance)

    def kick_full_power(self):
        distance=300
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
        super(RFCommsManager, self).kick_full_power()

    def close_grabbers(self):
        super(RFCommsManager, self).close_grabbers()

    def release_grabbers(self):
        super(RFCommsManager, self).release_grabbers()
