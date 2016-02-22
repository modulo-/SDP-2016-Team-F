import random
import thread
from threading import Timer, Lock, Condition
import b64
import serial
from logging import info, debug, error

ENCKEY = '3327BDBAAF48C59410FB5C4115777F26'
PANID = '6810'

DEVICEID = 'c'

class CommHandler:
    def init(self, fname, chan, control, listen=True):
        self._lock = Lock()
        self._callbacks = []
        self._packetlist = []
        self._packetcond = Condition()
        self._serial = serial.Serial(fname)
        self._serial.flushInput()
        self._serial.flushOutput()
        cmds = [
            control,
            'ATEE1\r',
            'ATAC\r',
            'ATEK' + ENCKEY + '\r',
            'ATID' + PANID + '\r',
            'ATCN' + str(chan) + '\r',
            'ATAC\r',
            'ATWR\r',
            'ATDN\r',
        ]

        for cmd in cmds:
            self._lock.acquire()
            self._serial.write(cmd)
            self._lock.release()
            debug('Comm command sent: %r', cmd)
            self._waitok()
        if listen:
            thread.start_new_thread(self._monitor, ())

    def _monitor(self):
        while True:
            line = self._serial.readline().strip()
            debug('Received raw comm input: %r', line)
            if len(line) < 4:
                continue
            if (not line.startswith(DEVICEID) and not
                    line.startswith('d') and not
                    line.startswith('e')):
                continue
            if line[1] == '$' and len(line) == 4:
                # ACK
                self._packetcond.acquire()
                index = None
                for (i, packet) in enumerate(self._packetlist):
                    if line[2:4] == packet[-4:-2]:
                        debug('Package ACK: %s', line)
                        index = i
                        break
                if index is not None:
                    self._packetlist.pop(index)
                if self._packetlist == []:
                    self._packetcond.notify()
                self._packetcond.release()
                continue
            if not b64.valid(line):
                continue
            if b64.checksum(line[:-2]) != line[-2:]:
                continue
            self._lock.acquire()
            self._serial.write(line[1] + '$' + line[-2:] + '\r\n')
            self._serial.flush()
            self._lock.release()
            data = b64.decode(line[2:-2])
            if line.startswith('d'):
                info('Debug message recieved: %s', data)
            elif line.startswith('e'):
                error('Error message recieved: %s', data)
            else:
                for callback in self._callbacks:
                    thread.start_new_thread(callback, (data, ))

    def _waitok(self):
        buf = (None, None)
        while buf != ('O', 'K'):
            buf = (buf[1], self._serial.read())

    def registercb(self, cb):
        self._callbacks.append(cb)

    def _check_recieved(self, packet):
        # TODO: Maybe limit number of retries? Or maybe that isn't desirable?
        self._packetcond.acquire()
        if packet in self._packetlist:
            self._lock.acquire()
            self._serial.write(packet)
            self._serial.flush()
            self._lock.release()
            Timer(get_standoff(), lambda: self._check_recieved(packet)).start()
        self._packetcond.release()

    def send(self, data, target):
        self._stop_resend(target)
        self.send_raw(data, target)

    def send_raw(self, data, target):
        packet = str(target) + DEVICEID + b64.encode(data)
        sum = b64.checksum(packet)
        packet += sum + '\r\n'
        self._packetcond.acquire()
        self._packetlist.append(packet)
        self._packetcond.release()
        self._lock.acquire()
        debug('Sent packet %r', packet)
        self._serial.write(packet)
        self._serial.flush()
        self._lock.release()
        Timer(get_standoff(), lambda: self._check_recieved(packet)).start()

    # Prevents any packets from being resent.
    #
    # Use the empty string as target to stop all resending.
    def _stop_resend(self, target):
        self._packetlist = [x for x in self._packetlist if not x.startswith(str(target))]

    def wait(self):
        self._packetcond.acquire()
        while self._packetlist != []:
            self._packetcond.wait()
        self._packetcond.release()

def get_standoff():
    return random.random() * 0.9 + 0.1
