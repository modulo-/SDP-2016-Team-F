#!/usr/bin/python2
from __future__ import division
import comms
import sys
from math import ceil, atan, sqrt, pi
import logging
from logging import debug, error
import readline

CMD_WAIT   = 0x00
CMD_BRAKE  = 0x01
CMD_STRAIT = 0x02
CMD_SPIN   = 0x03
CMD_KICK   = 0x04
CMD_MV     = 0x05

class TractorCrabException(Exception):
    def __init__(self, s):
        super(Exception, self).__init__(s)

class Command:
    def __init__(self, f, desc, argl):
        self.f = f
        self.desc = desc
        self.argl = argl

def i16fn(opcode):
    return lambda x: [opcode, x & 0xff, (x >> 8) & 0xff]

cmd_wait = i16fn(CMD_WAIT);
cmd_brake = i16fn(CMD_BRAKE)
cmd_strait = i16fn(CMD_STRAIT)
cmd_spin = i16fn(CMD_SPIN)
cmd_kick = i16fn(CMD_KICK)

def cmd_mv(x, y, angle):
    return [CMD_MV,
        x & 0xff, (x >> 8) & 0xff,
        y & 0xff, (y >> 8) & 0xff,
        angle & 0xff, (angle >> 8) & 0xff]

def parse_t(s):
    t = int(s)
    if t <= 0:
        raise TractorCrabException('Time must be > 0!')
    elif t > 0xffff:
        raise TractorCrabException('Time out of range!')
    return t

def parse_dist(s):
    d = int(float(s) * 10)
    if d < -0x8000 or d > 0x7fff:
        raise TractorCrabException('Distance out of range!')
    return d

def parse_angle(s):
    d = int(float(s) * 60)
    if d < -0x8000 or d > 0x7fff:
        raise TractorCrabException('Angle out of range!')
    return d

commands = {
    'wait': Command(cmd_wait, 'waits for the specified time',
        [('time', parse_t)]),
    'brake': Command(cmd_brake,
        'brakes for the specified time',
        [('time', parse_t)]),
    'strait': Command(cmd_strait,
        'moves strait (positive: right) the specified distance',
        [('dist', parse_dist)]),
    'spin': Command(cmd_spin,
        'Spins on the spot the specified angle (positive: clockwise)',
        [('angle', parse_angle)]),
    'kick': Command(cmd_kick,
        'Kicks the ball the specified distance',
        [('dist', parse_dist)]),
    'mv': Command(cmd_mv,
        'Moves to a x and y offset, facing a specified angle',
        [('x', parse_dist), ('y', parse_dist), ('angle', parse_angle)]),
}

def help_cmd(cmd):
    if cmd not in commands:
        print "No entry for '{}'.".format(cmd)
        return
    parts = [cmd]
    for arg in commands[cmd].argl:
        parts.append('<' + arg[0] + '>')
    print ' '.join(parts)
    print ''
    print commands[cmd].desc

def help_cmds():
    print "Commands available:"
    print ""
    cmdnames = list(commands.keys())
    cmdnames.extend(['help', 'exit'])
    cmdnames.sort()
    print ', '.join(cmdnames)
    print ""
    print "Run 'help <cmd>' for more details about '<cmd>'."
    print "Commands are run in a shell-like style, and take arguments"
    print "sererated with spaces. (e.g. 'wait 100')"
    print ""
    print "Commands take a fixed number of arguments, and may be chained in"
    print "sequence (e.g. strait 100 strait -100)."
    print ""
    print "Distances are measured in mm, angles in degrees and times in ms."
    print ""
    print "Commands which cannot be used in a chain are 'help' and 'exit'."

if __name__ == '__main__':
    logging.root.setLevel(logging.DEBUG)
    if len(sys.argv) < 2:
        error('RF device not specified')
        print "Please provide the RF device as an argument (e.g. '/dev/ttyACM0')"
    else:
        #comms.init(sys.argv[1], '60', '+++')
        print "Comms system online. Type 'help' for command information."
        print ""
        print "time is given in milliseconds. Please wait for a command to complete before sending a new one."
        print "commands aside from 'exit' may be chained. E.g.:"
        print ""
        print "kicker_extend_t 500 wait_t 1000 kicker_retract_t 500"
        print ""
        print "Please use this responsibly, packets are limited in effective size!"
        while True:
            try:
                line = raw_input('% ')
                args = line.split()
                i = 0
                if len(args) == 0:
                    continue
                elif args[0] == 'exit':
                    exit(0)
                elif args[0] == 'help':
                    if(len(args) >= 2):
                        help_cmd(args[1])
                    else:
                        help_cmds()
                    continue
                cmdbytes = []
                while i < len(args):
                    if args[i] not in commands:
                        raise TractorCrabException("Unknown command: '" + args[i] + "'")
                    cmd = commands[args[i]]
                    cmd_arglen = len(cmd.argl)
                    if len(args) < cmd_arglen + i + 1:
                        raise TractorCrabException('Too few arguments!')
                    cmd_args = args[i + 1 : i + 1 + cmd_arglen]
                    parsed_args = []
                    for (argspec, arg) in zip(cmd.argl, cmd_args):
                        parsed_args.append(argspec[1](arg))
                    cmdbytes.extend(cmd.f(*parsed_args))
                    i += cmd_arglen + 1
                if cmdbytes:
                    debug('Sent command sequence %r', cmdbytes)
                    comms.send(''.join(chr(x) for x in cmdbytes), '1')
            except TractorCrabException as e:
                error('%s', e)

