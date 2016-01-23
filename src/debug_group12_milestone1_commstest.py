import math
import comms

chunk_size = 30
serial_device = "COM18"  # or on linux /dev/ttyACM0 etc
robot_id = 2 # group 11 - 1, group 12 - 2
rf_channel = "67"  # hex
guard_chars = "~~~"
i2c_freq = 20  # MHz
filename = "b64.py"  # file to transmit


def main():
    # comms.init(serial_device, rf_channel, guard_chars, listen=True)
    with open(filename, mode='rb') as infile:
        data = infile.read(250)

    chunk_count = int( math.ceil(len(data)*1.0/chunk_size))

    # d[1B packet num][1B i2c freq][1B data len]
    cmd = b"d\x00" + chr(i2c_freq) + chr(len(data))
    # comms.send(cmd, 2)
    print ":".join("{:02x}".format(ord(c)) for c in cmd)

    for i in range(chunk_count):
        # d[1B packet num][1B index of first byte in packet][1B packet len][data]
        data_size = chunk_size if i != chunk_count-1 else len(data)%chunk_size
        first_byte = i*chunk_size
        data_str = b"d" + chr(i+1) + chr(first_byte) + chr(data_size) + "".join(data[first_byte:first_byte+data_size])
        print ":".join("{:02x}".format(ord(c)) for c in data_str)
        # comms.send(data_str, robot_id)

if __name__ == '__main__':
    main()
