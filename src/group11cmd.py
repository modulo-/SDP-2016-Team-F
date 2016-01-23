#/usr/bin/python2
import comms
import sys

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
    while True:
        line = raw_input('% ')
        if line.startswith("led_on "):
            time = int(line[7:])
            if time > 10000:
                print "time given greater than 10 secs. ignoring."
            comms.send(''.join(chr(x) for x in [0, time & 0xff, (time >> 8) & 0xff]), '1')

        elif line.startswith("kicker_fwd "):
            time = int(line[11:])
            if time > 10000:
                print "time given greater than 10 secs. ignoring."
            comms.send(''.join(chr(x) for x in [1, time & 0xff, (time >> 8) & 0xff]), '1')
        elif line.startswith("kicker_bwd "):
            time = int(line[11:])
            if time > 10000:
                print "time given greater than 10 secs. ignoring."
            comms.send(''.join(chr(x) for x in [1, time & 0xff, (time >> 8) & 0xff]), '1')
        elif line == 'exit':
            exit(0)

