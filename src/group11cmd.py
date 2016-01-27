#!/usr/bin/python2
from __future__ import division
import comms
import sys
from math import ceil, atan, sqrt, pi

def rotation_time_from_angle(angle):
    return int(200 + 21 * angle / 3)

def move_time_from_distance(dist):
    return int(200 + 100 * dist / 3)

def kick_time_from_distance(dist):
    if dist >= 150:
        return 350
    else:
        return int(170 + 18 * dist / 50)

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
    for i in range(int(ceil(float(len(cont)) / 25))):
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
    print "left_t <time>"
    print "right_t <time>"
    print "spin_cw_t <time>"
    print "spin_cc_t <time>"
    print "left <dist>"
    print "right <dist>"
    print "spin_cw <dist>"
    print "spin_cc <dist>"
    print "kick_t <time>"
    print "kick <dist>"
    print "mv <x> <y> <angle>"
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
        print repr(args)
        i = 0
        if len(args) == 0:
            continue
        if args[0] == 'send_file':
            send_file(int(args[1]), args[2])
            continue
        cmdbytes = []
        while i < len(args):
            if args[i] in ['led_on', 'kicker_fwd', 'kicker_bwd', 'left_t',
                    'right_t', 'spin_cw_t', 'spin_cc_t']:
                cmd = None
                if args[i] == 'led_on':
                    cmd = 0
                elif args[i] == 'kicker_fwd':
                    cmd = 1
                elif args[i] == 'kicker_bwd':
                    cmd = 2
                elif args[i] == 'right_t':
                    cmd = 3
                elif args[i] == 'left_t':
                    cmd = 4
                elif args[i] == 'spin_cc_t':
                    cmd = 5
                elif args[i] == 'spin_cw_t':
                    cmd = 6
                time = guardedint(args[i + 1])
                if time != -1:
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff])
                i += 2
            elif args[i] in ['left', 'right']:
                cmd = None
                if args[i] == 'right':
                    cmd = 3
                elif args[i] == 'left':
                    cmd = 4
                dist = guardedint(args[i + 1])
                time = move_time_from_distance(dist)
                if dist != -1:
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff])
                i += 2
            elif args[i] in ['spin_cw', 'spin_cc']:
                cmd = None
                if args[i] == 'spin_cc':
                    cmd = 5
                elif args[i] == 'spin_cw':
                    cmd = 6
                angle = guardedint(args[i + 1])
                time = rotation_time_from_angle(angle)
                if angle != -1:
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff])
                i += 2
            elif args[i] == 'kick_t':
                time = guardedint(args[i + 1])
                cmdbytes.extend([0x02, time & 0xff, (time >> 8) & 0xff])
                # Led on 100ms (basically a wait)
                cmdbytes.extend([0x00, 0x64, 0x00])
                # Kicker forward 270 ms
                cmdbytes.extend([0x01, 0x0e, 0x01])
                i += 2
            elif args[i] == 'kick':
                dist = guardedint(args[i + 1])
                time = kick_time_from_distance(dist)
                cmdbytes.extend([0x02, time & 0xff, (time >> 8) & 0xff])
                # Led on 100ms (basically a wait)
                cmdbytes.extend([0x00, 0x64, 0x00])
                # Kicker forward 270 ms
                cmdbytes.extend([0x01, 0x0e, 0x01])
                i += 2
            elif args[i] == 'mv':
                x = int(args[i + 1])
                y = int(args[i + 2])
                angle = int(args[i + 3])
                mv_angle = None
                if x == 0:
                    mv_angle = 90
                else:
                    mv_angle = -atan(y/x) * 180.0 / pi
                dist = sqrt(x**2 + y**2)
                if x <= 0:
                    dist = -dist
                face_angle = (angle - mv_angle) % 360
                if face_angle > 180:
                    face_angle -= 360
                print x, y, angle, mv_angle, dist, face_angle
                # Rotate mv_angle
                # Move dist
                # Rotate face_angle
                if mv_angle != 0:
                    cmd = None
                    if mv_angle > 0:
                        cmd = 0x06
                    else:
                        cmd = 0x05
                    time = rotation_time_from_angle(abs(mv_angle))
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff,
                        0x00, 0x64, 0x00])
                if dist != 0:
                    cmd = None
                    if dist < 0:
                        cmd = 0x03
                    else:
                        cmd = 0x04
                    time = move_time_from_distance(abs(dist))
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff,
                        0x00, 0x64, 0x00])
                if face_angle != 0:
                    cmd = None
                    if face_angle > 0:
                        cmd = 0x06
                    else:
                        cmd = 0x05
                    time = rotation_time_from_angle(abs(face_angle))
                    cmdbytes.extend([cmd, time & 0xff, (time >> 8) & 0xff,
                        0x00, 0x64, 0x00])
                i += 3
            else:
                i += 1
        comms.send(''.join(chr(x) for x in cmdbytes), '1')

