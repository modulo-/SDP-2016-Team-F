#!/usr/bin/python2
from __future__ import division
import comms
import sys
from math import ceil, atan, sqrt, pi
import logging
from logging import debug, error
import readline

CMD_WAIT_T           = 0x00
CMD_KICKER_RETRACT_T = 0x01
CMD_KICKER_EXTEND_T  = 0x02
CMD_RIGHT_T          = 0x03
CMD_LEFT_T           = 0x04
CMD_SPIN_CC_T        = 0x05
CMD_SPIN_CW_T        = 0x06
CMD_BRAKE_T          = 0x07
CMD_MV_STRAIT        = 0x80
CMD_SPIN             = 0x81
CMD_KICK             = 0x82
CMD_MV               = 0x83

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

cmd_wait_t = i16fn(CMD_WAIT_T);
cmd_kicker_extend_t = i16fn(CMD_KICKER_EXTEND_T)
cmd_kicker_retract_t = i16fn(CMD_KICKER_RETRACT_T)
cmd_left_t = i16fn(CMD_LEFT_T)
cmd_right_t = i16fn(CMD_RIGHT_T)
cmd_spin_cc_t = i16fn(CMD_SPIN_CC_T)
cmd_spin_cw_t = i16fn(CMD_SPIN_CW_T)
cmd_brake_t = i16fn(CMD_BRAKE_T)
cmd_mv_strait = i16fn(CMD_MV_STRAIT)
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
    'wait_t': Command(cmd_wait_t, 'waits for the specified time',
        [('time', parse_t)]),
    'kicker_extend_t': Command(cmd_kicker_extend_t,
        'extends the kicker for the specified time',
        [('time', parse_t)]),
    'kicker_retract_t': Command(cmd_kicker_retract_t,
        'retracts the kicker for the specified time',
        [('time', parse_t)]),
    'left_t': Command(cmd_left_t,
        'moves left for the specified time',
        [('time', parse_t)]),
    'right_t': Command(cmd_right_t,
        'moves right for the specified time',
        [('time', parse_t)]),
    'spin_cw_t': Command(cmd_spin_cw_t,
        'spins clockwise for the specified time',
        [('time', parse_t)]),
    'spin_cc_t': Command(cmd_spin_cc_t,
        'spins counter-clockwise for the specified time',
        [('time', parse_t)]),
    'brake_t': Command(cmd_brake_t,
        'brakes for the specified time',
        [('time', parse_t)]),
    'mv_strait': Command(cmd_mv_strait,
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
    # TODO
    pass

def help_cmds():
    # TODO
    print "Commands available:"
    print ""
    print "wait, kicker_ext_t, kicker_ret_t, left_t, right_t, spin_cw_t,"
    print "spin_cc_t, kick_t, left, right, spin_cw, spin_cc, kick, mv, exit,"
    print "help."
    print ""
    print "Run 'help <cmd>' for more details about '<cmd>'."
    print "Commands are run in a shell-like style, and take arguments"
    print "sererated with spaces. (e.g. 'wait 100')"
    print ""
    print "Commands take a fixed number of arguments, and may be chained in"
    print "sequence (e.g. left 1000 right 1000)."
    print ""
    print "Commands which cannot be used in a chain are 'help' and 'exit'."

if __name__ == '__main__':
    logging.root.setLevel(logging.DEBUG)
    if len(sys.argv) < 2:
        error('RF device not specified')
        print "Please provide the RF device as an argument (e.g. '/dev/ttyACM0')"
    else:
        comms.init(sys.argv[1], '60', '+++')
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

