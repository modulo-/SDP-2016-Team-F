import os
import thread
from threading import Timer
import b64
import serial

ENCKEY = '3327BDBAAF48C59410FB5C4115777F26'
PANID = '6810'

DEVICEID = 'c'

commserial = None
callbacks = []
# List of sent and not-yet acknowledged packets.
packetlist = []

def monitor_comms():
    while True:
        line = commserial.readline()
        if len(line) < 6:
            continue
        if not line.startswith(DEVICEID):
            continue
        if line[1] == '$' and len(line) == 6:
            # ACK
            print line
            index = None
            for (i, packet) in enumerate(packetlist):
                if line[2:4] == packet[-4:-2]:
                    print "ACK"
                    index = i
                    break;
            if index != None:
                packetlist.pop(index)
            continue
        if not b64.valid(line[:-1]):
            continue
        if b64.checksum(line[:-3]) != line[-4:-2]:
            continue
        commserial.write(line[1] + '$' + line[-4:-2] + '\r\n')
        #os.fsync(commfd)
        data = b64.decode(line[2:-4])
        # TODO: how to pass data to callbacks without blocking the monitor
        # thread?

def waitok():
    #os.fsync(commfd)
    buf = (None, None)
    while buf != ('O', 'K'):
        buf = (buf[1], commserial.read())

def init(fname, chan, control):
    global commserial
    commserial = serial.Serial(fname)
    commserial.flushInput()
    commserial.flushOutput()
    cmds = [
        control,
        'ATEE1\r',
        'ATAC\r',
        'ATEK' + ENCKEY + '\r',
        'ATID' + PANID + '\r',
        'ATCN' + chan + '\r',
        'ATAC\r',
        'ATDN\r',
    ]
    for cmd in cmds:
        commserial.write(cmd)
        waitok()
    thread.start_new_thread(monitor_comms, ())

def registercb(cb):
    callbacks.append(cb)

def check_recieved(packet):
    # TODO: Maybe limit number of retries? Or maybe that isn't desirable?
    if packet in packetlist:
        commserial.write(packet)
        #os.fsync(commfd)
        Timer(0.01, lambda: check_recieved(packet)).start()

def send(data, target):
    packet = target + DEVICEID + b64.encode(data)
    sum = b64.checksum(packet)
    packet += sum + '\r\n'
    print repr(packet)
    packetlist.append(packet)
    commserial.write(packet)
    Timer(0.01, lambda: check_recieved(packet)).start()

# Prevents any packets from being resent.
#
# Example use: To prevent old command sequences from being sent when a new one
# will be issued.
#     stop_resend(target)
#     send(new_data, target)
#
# Use the empty string as target to stop all resending.
def stop_resend(target):
    global packetlist
    packetlist = [x for x in packetlist if not x.startswith(target)]
