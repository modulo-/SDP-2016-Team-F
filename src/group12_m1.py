import math
import struct
import time

import comms


serial_device = "COM16"  # or on linux /dev/ttyACM0 etc
robot_id = 2 # group 11 - 1, group 12 - 2
rf_channel = "67"  # hex
guard_chars = "~~~"
chunk_size = 30



def doMove():
    x = int(raw_input("X: "))
    y = int(raw_input("Y: "))
    heading = math.degrees(math.atan2(y, x))
    distance = math.sqrt(x**2+y**2)

    finalHeading = int(raw_input("Final Heading: "))

    cmd = b"k"+struct.pack(">h", heading)+struct.pack(">h", distance)+struct.pack(">h", finalHeading)
    comms.send(cmd, robot_id)

def doKick():
    distance = int(raw_input("Distance to kick: "))
    cmd = b"k"+struct.pack(">h", distance)
    comms.send(cmd, robot_id)

def doData():
    filename = raw_input("File to transmit: ")
    i2c_freq = int(raw_input("Frequency on i2c: "))
    with open(filename, mode='rb') as infile:
        data = infile.read(255)

    chunk_count = int( math.ceil(len(data)*1.0/chunk_size))

    # d[1B packet num][1B i2c freq][1B data len]
    cmd = b"d\x00" + struct.pack(">B", i2c_freq) + struct.pack(">B", len(data))
    comms.send(cmd, robot_id)
    # print ":".join("{:02x}".format(ord(c)) for c in cmd)

    for i in range(chunk_count):
        # time.sleep(1)
        # d[1B packet num][1B index of first byte in packet][1B packet len][data]
        data_size = chunk_size if i != chunk_count-1 else len(data)%chunk_size
        first_byte = i*chunk_size
        data_str = b"d" + struct.pack(">B", i+1) + struct.pack(">B", first_byte) + struct.pack(">B", data_size) + b"".join(data[first_byte:first_byte+data_size])
        # print ":".join("{:02x}".format(ord(c)) for c in data_str)
        comms.send(data_str, robot_id)

    while (comms.packetlist != []):
        time.sleep(1)

    # d[1B FF]
    cmd = b"d\xff"
    comms.send(cmd, robot_id)


def main():
    # comms.init(serial_device, rf_channel, guard_chars, listen=True)

    while (True):
        try:
            command = raw_input("Command to execute (m/k/d) or q to quit")
            if command == "m":
                doMove()
            if command == "k":
                doKick()
            if command == "d":
                doData()
            if command == "q":
                break

        except ValueError:
            pass
if __name__ == '__main__':
    main()
