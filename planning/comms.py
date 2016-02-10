import math
import rf_comms as comms
import struct

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

class RFCommsManager (CommsManager):

    def __init__(self, robot, serial_device):
        # group 11 - 1, group 12 - 2
        self.robot_id = 2
        # hex
        rf_channel = "67"
        guard_chars = "~~~"
        chunk_size = 30
        comms.init(serial_device, rf_channel, guard_chars, listen=True)
        super(RFCommsManager, self).__init__(robot)

    # move a distance in mm
    def move(self, distance):
        cmd = b"m"+struct.pack(">h", distance)
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).move(distance)
    
    # turn by an angle in degrees
    def turn(self, angle):
        cmd = b"t"+struct.pack(">h", math.degrees(angle))
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).turn(angle)
    
    # kick a distance in cm
    def kick(self, distance):
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).kick(distance)

    def kick_full_power(self):
        distance=300
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).kick_full_power()

    def close_grabbers(self):
        cmd = b"g"
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).close_grabbers()

    def release_grabbers(self):
        cmd = b"r"
        comms.send(cmd, self.robot_id)
        super(RFCommsManager, self).release_grabbers()
