import os
import thread

ENCKEY = '3327BDBAAF48C59410FB5C4115777F26'
PANID = '6810'

DEVICEID = 'c'

commfd = None
commfile = None
callbacks = []
# List of sent and not-yet acknowledged packets.
packetlist = []

def monitor_comms():
    for line in commfile:
        if len(line) < 5:
            continue
        if not line.startswith(DEVICEID):
            continue
        if line[1] == '$' and len(line) == 5:
            # ACK
            index = None
            for (i, packet) in packetlist:
                if line[2:4] == packet[-3:-1]:
                    index = i
                    break;
            if index:
                del packetlist[index]
            continue
        if not base64.valid(line[:-1]):
            continue
        if base64.checksum(line[:-3]) != line[-3:-1]:
            continue
        os.write(commfd, line[1] + '$' + line[-3:-1] + '\n')
        #os.fsync(commfd)
        data = b64.decode(line[2:-3])
        # TODO: how to pass data to callbacks without blocking the monitor
        # thread?

def waitok():
    #os.fsync(commfd)
    buf = (None, None)
    while buf != ('O', 'K'):
        buf = (buf[1], os.read(commfd, 1))

def init(fname, chan, control):
    global commfd, commfile
    commfd = os.open(fname, os.O_RDWR)
    commfile = os.fdopen(commfd, 'r')
    cmds = [
        control,
        'ATEE1\n',
        'ATAC\n',
        'ATEK' + ENCKEY + '\n',
        'ATID' + PANID + '\n',
        'ATCN' + chan + '\n',
        'ATAC\n',
        'ATDN\n',
    ]
    for cmd in cmds:
        os.write(commfd, cmd)
        waitok()
    thread.start_new_thread(monitor_comms, ())

def registercb(cb):
    callbacks.append(cb)

def check_recieved(packet):
    # TODO: Maybe limit number of retries? Or maybe that isn't desirable?
    if data in packetlist:
        os.write(commfd, packet)
        #os.fsync(commfd)
        Timer(0.01, lambda: check_recieved(packet)).start()

def send(data, target):
    packet = target + DEVICEID + b64.encode(data)
    sum = b64.checksum(data)
    packet += sum + '\n'
    packetlist.append(packet)
    os.write(commfd, packet)
    #os.fsync(commfd)
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
