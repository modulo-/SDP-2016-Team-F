#!/usr/bin/python2
import comms
import sys

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



if len(sys.argv) < 2:
    print "Please provide the RF device as an argument (e.g. '/dev/ttyACM0')"
else:
    comms.init(sys.argv[1], '60', '+++')
    print "Comms system online. The following commands are supported:"
    print ""
    print "led_on <time>"
    print "kicker_fwd <time>"
    print "kicker_bwd <time>"
    print "exit"
    print ""
    print "time is given in milliseconds. Please wait for a command to complete before sending a new one."
    print "commands aside from 'exit' may be chained. E.g.:"
    print ""
    print "kicker_fwd 500 led_on 1000 kicker_bwd 500"
    print ""
    print "Please use this responsibly, packets are limited in effective size!
    while True:
        line = raw_input('% ')
        if line == 'exit':
            exit(0)
        args = line.split()
        i = 0
        cmdbytes = []
        while i < len(args):
            if args[i] == 'led_on':
                time = guardedint(args[i + 1])
                if time != -1:
                    cmdbytes.extend([0, time & 0xff, (time >> 8) & 0xff])
                i += 2
            elif args[i] == 'kicker_fwd':
                time = guardedint(args[i + 1])
                if time != -1:
                    cmdbytes.extend([1, time & 0xff, (time >> 8) & 0xff])
                i += 2
            elif args[i] == 'kicker_bwd':
                time = guardedint(args[i + 1])
                if time != -1:
                    cmdbytes.extend([2, time & 0xff, (time >> 8) & 0xff])
                i += 2
        comms.send(''.join(chr(x) for x in cmdbytes), '1')

