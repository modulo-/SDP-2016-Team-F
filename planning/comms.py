import math
import rf_comms


class CommsManager(object):
        self.robot_index = robot
        robot_id = 2 # group 11 - 1, group 12 - 2
        rf_channel = "67"  # hex
        guard_chars = "~~~"
        chunk_size = 30

    def __init__(self, robot, serial_device):
        comms.init(serial_device, rf_channel, guard_chars, listen=True)

    #move a distance in mm
    def move(self, distance):
        print("Moving robot {0} distance {1}".format(self.robot_index, distance))
        cmd = b"m"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
    
    #move an angle in degrees 
    def turn(self, angle):
        print("Turning robot {0} angle {1}".format(self.robot_index, angle))
        cmd = b"t"+struct.pack(">h", angle)
        comms.send(cmd, robot_id)
    
    #kick a distance in cm
    def kick(self, distance):
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
        print("Robot {0} kicking distance {1}".format(self.robot_index, distance))

    def kick_full_power(self):
        distance=300
        cmd = b"k"+struct.pack(">h", distance)
        comms.send(cmd, robot_id)
        print("Robot {0} kicking full power".format(self.robot_index))

    def close_grabbers(self):
        print("Robot {0} closing grabbers".format(self.robot_index))

    def release_grabbers(self):
        print("Robot {0} releasing grabbers".format(self.robot_index))
