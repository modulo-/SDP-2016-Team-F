import os
import thread
from threading import Timer, Lock, Condition
import b64
import serial
from logging import info, debug

ENCKEY = '3327BDBAAF48C59410FB5C4115777F26'
PANID = '6810'

DEVICEID = 'c'

commserial = None
commlock = Lock()
callbacks = []
# List of sent and not-yet acknowledged packets.
packetlist = []
packetcond = Condition()

def monitor_comms():
    while True:
        line = commserial.readline().strip()
        if len(line) < 4:
            continue
        if not line.startswith(DEVICEID) and not line.startswith('d'):
            continue
        if line[1] == '$' and len(line) == 4:
            # ACK
            packetcond.acquire()
            index = None
            for (i, packet) in enumerate(packetlist):
                if line[2:4] == packet[-4:-2]:
                    debug('Package ACK: %s', line)
                    index = i
                    break;
            if index != None:
                packetlist.pop(index)
            if packetlist == []:
                packetcond.notify()
            packetcond.release()
            continue
        if not b64.valid(line):
            continue
        if b64.checksum(line[:-2]) != line[-2:]:
            continue
        commlock.acquire()
        commserial.write(line[1] + '$' + line[-2:] + '\r\n')
        commserial.flush()
        commlock.release()
        data = b64.decode(line[2:-2])
        if line.startswith('d'):
            info('Debug message recieved: %s', data)
        else:
            for callback in callbacks:
                thread.start_new_thread(callback, (data, ))

def waitok():
    buf = (None, None)
    while buf != ('O', 'K'):
        buf = (buf[1], commserial.read())

def init(fname, chan, control, listen=True):
    global commserial
    commserial = serial.Serial(fname)
    commserial.reset_input_buffer()
    commserial.reset_output_buffer()
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
        commlock.acquire()
        commserial.write(cmd)
        commlock.release()
        waitok()
    if listen:
        thread.start_new_thread(monitor_comms, ())

def registercb(cb):
    callbacks.append(cb)

def check_recieved(packet):
    # TODO: Maybe limit number of retries? Or maybe that isn't desirable?
    packetcond.acquire()
    if packet in packetlist:
        commlock.acquire()
        commserial.write(packet)
        commserial.flush()
        commlock.release()
        Timer(0.1, lambda: check_recieved(packet)).start()
    packetcond.release()

def send(data, target):
    debug('Sent: %r', data)
    packet = target + DEVICEID + b64.encode(data)
    sum = b64.checksum(packet)
    packet += sum + '\r\n'
    packetcond.acquire()
    packetlist.append(packet)
    packetcond.release()
    commlock.acquire()
    commserial.write(packet)
    commserial.flush()
    commlock.release()
    Timer(0.1, lambda: check_recieved(packet)).start()

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

def wait():
    packetcond.acquire()
    while packetlist != []:
        packetcond.wait()
    packetcond.release()
