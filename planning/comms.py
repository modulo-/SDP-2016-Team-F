import math
from rf_comms import SerialHandle
import struct
from logging import info


class CommsManager(object):

    def __init__(self, robot):
        self.robot_index = robot

    def move(self, distance):
        info("Moving robot {0} distance {1}".format(self.robot_index, distance))

    def turn(self, angle):
        info("Turning robot {0} angle {1} radians ( {2} degrees)".format(self.robot_index, angle, math.degrees(angle)))

    def kick(self, distance):
        info("Robot {0} kicking distance {1}".format(self.robot_index, distance))

    def kick_full_power(self):
        info("Robot {0} kicking full power".format(self.robot_index))

    def close_grabbers(self):
        info("Robot {0} closing grabbers".format(self.robot_index))

    def release_grabbers(self):
        info("Robot {0} releasing grabbers".format(self.robot_index))


class TractorCrabCommsManager(CommsManager):
    CMD_WAIT          = 0x00
    CMD_BRAKE         = 0x01
    CMD_STRAIT        = 0x02
    CMD_SPIN          = 0x03
    CMD_KICK          = 0x04
    CMD_MV            = 0x05
    CMD_GRABBER_OPEN  = 0x06
    CMD_GRABBER_CLOSE = 0x07
    CMD_HOLD_SPIN     = 0x08

    def __init__(self, robot, serial_device):
        self.robot_id = 1
        self._handle = SerialHandle(serial_device, "60", "~~~")
        CommsManager.__init__(self, robot)

    def _run(self, cmd):
        self._handle.send(''.join(chr(i) for i in cmd), '1')

    def _normalize_angle(self, angle):
        # Expects inputs in radians counter-clockwise.
        # Returns output in minutes clockwise.
        angle = int(math.degrees(angle) * 60)
        if angle < -10800 or angle > 10800:
            angle = (angle + 10800) % 21600 - 10800
        return angle

    def _normalize_dist(self, distance):
        # Expects inputs in px (1px ~= 0.5cm. Returns output in mm.
        dist = int(distance * 5)
        # Ensure that dist is always in range. Set to 0 else.
        if dist < -0x8000 or dist > 0x7fff:
            dist = 0
        return dist

    def _16_bitify(self, n):
        return [n & 0xff, (n >> 8) & 0xff]

    def turn_then_move(self, angle, distance):
        self._run(
            [self.CMD_SPIN] +
            self._16_bitify(self._normalize_angle(angle)) +
            [self.CMD_STRAIT] +
            self._16_bitify(self._normalize_dist(distance)))

    def turn_then_kick(self, angle):
        self._run(
            [self.CMD_SPIN] +
            self._16_bitify(self._normalize_angle(angle)) +
            [self.CMD_KICK] +
            self._16_bitify(100))

    def move(self, distance):
        # NOTE: moves to the right, NOT forward. (We need better support for a different action set).
        # Expects input in cm.
        # Accepts negative input.
        # May fail or out-of-range inputs, this gets silently swallowed.
        self._run([self.CMD_STRAIT] + self._16_bitify(self._normalize_dist(distance)))
        CommsManager.move(self, distance)

    def turn(self, angle):
        # NOTE: spins counter-clockwise, expects input in radians.
        # Accepts negative input.
        # May fail or out-of-range inputs, this gets silently swallowed.
        self._run([self.CMD_SPIN] + self._16_bitify(self._normalize_angle(angle)))
        CommsManager.turn(self, angle)

    def kick(self, distance):
        # NOTE: currently, functions the same as a full-power kick.
        self._run([self.CMD_GRABBER_OPEN, self.CMD_KICK] + self._16_bitify(100))
        CommsManager.kick(self, distance)

    def kick_full_power(self):
        self._run([self.CMD_GRABBER_OPEN, self.CMD_KICK] + self._16_bitify(100))
        CommsManager.kick_full_power(self)

    def close_grabbers(self):
        self._run([self.CMD_GRABBER_CLOSE])
        CommsManager.close_grabbers(self)

    def release_grabbers(self):
        self._run([self.CMD_GRABBER_OPEN])
        CommsManager.release_grabbers(self)


class RFCommsManager (CommsManager):

    def __init__(self, robot, serial_device, grab_callback):
        # group 11 - 1, group 12 - 2
        self.robot_id = 2
        # hex
        rf_channel = "67"
        guard_chars = "~~~"
        self._handle = SerialHandle(serial_device, rf_channel, guard_chars)
        self._handle.registercb(self.update_grabbers)
        self.grab_callback = grab_callback
        super(RFCommsManager, self).__init__(robot)

    def update_grabbers(data, somestuff):
        if data=="NC":
            self.grab_callback(False)
            info("Ball not caught")
        elif data=="BC":
            self.grab_callback(True)
            info("Ball caught")

    # move a distance in mm
    def move(self, distance):
        mm_distance = distance / 0.25 #0.1958
        cmd = b"m" + struct.pack(">h", mm_distance)
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).move(mm_distance)

    # turn by an angle in degrees
    def turn(self, angle):
        cmd = b"t" + struct.pack(">h", math.degrees(angle))
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).turn(angle)

    # kick a distance in cm
    def kick(self, distance):
        cmd = b"k" + struct.pack(">h", distance)
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).kick(distance)

    def kick_full_power(self):
        distance = 300
        cmd = b"k" + struct.pack(">h", distance)
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).kick_full_power()

    def close_grabbers(self):
        cmd = b"g"
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).close_grabbers()

    def release_grabbers(self):
        cmd = b"r"
        self._handle.send(cmd, self.robot_id)
        super(RFCommsManager, self).release_grabbers()
    
    def ping(self):
        cmd = b"p"
        self._handle.send(cmd, self.robot_id)

    def test(self):
        cmd = b"t"
        self._handle.send(cmd, self.robot_id)

    def setDebugLevel(self, level):
        cmd = b"D" + struct.pack(">h", level)
        self._handle.send(cmd, self.robot_id)
