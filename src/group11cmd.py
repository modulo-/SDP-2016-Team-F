#!/usr/bin/python2
import comms
import sys
import math

def guardedint(arg):
    ret = int(arg)
    if ret > 10000:
        print "time given greater than 10 secs. ignoring."
        return -1
    elif ret < 0:
        print "stop trying to break my code you asshole."
        return -1
    else:
        return ret

def echocb(data):
    print repr(data)

def send_file(delay, f):
    f = open(f, 'rb')
    cont = f.read()
    f.close()
    comms.send(''.join(chr(x) for x in [0xf0, len(cont)]), '1')
    comms.send(''.join(chr(x) for x in [0xf1, delay & 0xff, (delay >> 8) & 0xff]), '1')
    comms.wait()
    for i in range(int(math.ceil(float(len(cont)) / 25))):
        data = chr(0x80 | i) + cont[i * 25 : (i+1) * 25]
        data += chr(0x00) * (26 - len(data))
        print ">>>", repr(data)
        comms.send(data, '1')
        comms.wait()
    comms.send(chr(0xf2), '1')


if len(sys.argv) < 2:
    print "Please provide the RF device as an argument (e.g. '/dev/ttyACM0')"
else:
    comms.init(sys.argv[1], '60', '+++')
    comms.registercb(echocb)
    print "Comms system online. The following commands are supported:"
    print ""
    print "led_on <time>"
    print "kicker_fwd <time>"
    print "kicker_bwd <time>"
    print "left <time>"
    print "right <time>"
    print "spin_cw <time>"
    print "spin_cc <time>"
    print "send_file <delay> <f>"
    print "exit"
    print ""
    print "time is given in milliseconds. Please wait for a command to complete before sending a new one."
    print "commands aside from 'exit' and 'send_file' may be chained. E.g.:"
    print ""
    print "kicker_fwd 500 led_on 1000 kicker_bwd 500"
    print ""
    print "Please use this responsibly, packets are limited in effective size!"
    while True:
        line = raw_input('% ')
        if line == 'exit':
            exit(0)
        args = line.split()
        i = 0
        if args[0] == 'send_file':
            send_file(int(args[1]), args[2])
            continue
        cmdbytes = []
        while i < len(args):
            if args[i] in ['led_on', 'kicker_fwd', 'kicker_bwd', 'left',
                    'right', 'spin_cw', 'spin_cc']:
                cmd = None
                if args[i] == 'led_on':
                    cmd = 0
                elif args[i] == 'kicker_fwd':
                    cmd = 1
                elif args[i] == 'kicker_bwd':
                    cmd = 2
                elif args[i] == 'left':
                    cmd = 3
                elif args[i] == 'right':
                    cmd = 4
                elif args[i] == 'spin_cw':
                    cmd = 5
                elif args[i] == 'spin_cc':
                    cmd = 6
                time = guardedint(args[i + 1])
                if time != -1:
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff])
                i += 2
        comms.send(''.join(chr(x) for x in cmdbytes), '1')

