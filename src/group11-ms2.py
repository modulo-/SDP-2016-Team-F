#!/usr/bin/python2

from os import mkfifo, path
from tempfile import mkdtemp
from threading import Lock, Thread
from shutil import rmtree
from time import sleep
import json
import subprocess

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
    with open(visionpipe, 'r') as pipe:
        while True:
            line = pipe.readline()
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

team = 'y'
player = 'p'

mkfifo(visionpipe)
t = Thread(target=monitor_vision)
t2 = Thread(target=poll_plan)
t2.daemon = True
t.start()
t2.start()

print visionpipe
subprocess.call(['../vision/vision', visionpipe])

t.join()
rmtree(tmpdir)
