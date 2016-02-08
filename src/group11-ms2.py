#!/usr/bin/python2

from os import mkfifo, path
from tempfile import mkdtemp
from threading import Lock, Thread
from shutil import rmtree
from time import sleep
from sys import argv
import json
import subprocess
import readline

IDLE = 0
GRAB = 1
SPIN_GRAB = 2
KICK = 3

target = IDLE

class Robot:
    def __init__(self, x, y, facing):
        self.x = x
        self.y = y
        self.facing = facing

class Ball:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class State:
    def __init__(self, robots, ball):
        self.robots = robots
        self.ball = ball

tmpdir = mkdtemp()
visionpipe = path.join(tmpdir, 'visionpipe')

state = None
statelock = Lock()

def update_plan(state):
    # TODO: magic
    if target == GRAB:
        pass
    elif target == SPIN_GRAB:
        pass
    elif target == KICK:
        pass

def genrobot(obj):
    return Robot(obj['x'], obj['y'], obj['f'])

def genball(obj):
    return Ball(obj['x'], obj['y'])

def genstate(obj):
    dict = {}
    ball = None
    for k, v in obj.items():
        if len(k) == 2:
            dict[k] = genrobot(v)
        elif k == 'b':
            ball = genball(v)
    return State(dict, ball)

def monitor_vision():
    global state
    with open(visionpipe, 'r') as pipe:
        while True:
            line = pipe.readline().strip()
            # EOF
            if len(line) == 0:
                break
            obj = json.loads(line.strip())
            statelock.acquire()
            state = genstate(obj)
            statelock.release()

def poll_plan():
    while True:
        statelock.acquire()
        statecp = state
        statelock.release()
        update_plan(statecp)
        sleep(3)

def shell():
    global target
    while True:
        try:
            line = raw_input('% ')
            if line == 'exit':
                break
            elif line == 'grab':
                target = GRAB
            elif line == 'sgrab':
                target = SPIN_GRAB
            elif line == 'kick':
                target = KICK
            elif line == 'idle':
                target = IDLE
        except:
            break

player = argv[1]

mkfifo(visionpipe)
t = Thread(target=monitor_vision)
t.daemon = True
t2 = Thread(target=poll_plan)
t3 = Thread(target=shell)
t2.daemon = True
t.start()
t2.start()
t3.start()

subprocess.call(['../vision/vision', visionpipe])

t3.join()
rmtree(tmpdir)
