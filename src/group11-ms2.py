#!/usr/bin/python2

from __future__ import division
from os import mkfifo, path
from tempfile import mkdtemp
from threading import Lock, Thread
from shutil import rmtree
from time import sleep
from sys import argv
from planning import planner, world
from planning.position import Vector
from math import pi
import json
import subprocess
import readline
import comms
import group11cmd

def translate_pos(p):
    return p / 2.31

class CommsManager:
    def move(self, distance):
        group11cmd.run(group11cmd.cmd_strait(
            group11cmd.parse_dist(distance)))
    
    def turn(self, angle):
        group11cmd.run(group11cmd.cmd_spin(
            group11cmd.parseangle(angle / pi * 180)))
    
    def kick(self, distance):
        group11cmd.run(group11cmd.cmd_kick(100))
    
    def kick_full_power(self):
        group11cmd.run(group11cmd.cmd_kick(100))
    
    def close_grabbers(self):
        group11cmd.run(group11cmd.cmd_grabber_close())
    
    def release_grabbers(self):
        group11cmd.run(group11cmd.cmd_grabber_open())

planner = planner.Planner(CommsManager())
tmpdir = mkdtemp()
visionpipe = path.join(tmpdir, 'visionpipe')

# TODO: better way to initialize.
world = world.World('left', 0)
statelock = Lock()

def update_plan():
    planner.plan_and_act(world)

def updateworld(obj):
    robvec = world.our_robot.vector
    ballvec = world.ball.vector
    if player in obj:
        robvec = Vector(
            translate_pos(obj[player]['x']),
            -translate_pos(obj[player]['y']),
            obj[player]['f'] * pi / 180, 0)
    if 'b' in obj:
        ballvec = Vector(
            translate_pos(obj['b']['x']),
            -translate_pos(obj['b']['y']), 0, 0)
    world.update_positions({
        'our_robot': robvec,
        'ball': ballvec,
    })

def monitor_vision():
    with open(visionpipe, 'r') as pipe:
        while True:
            line = pipe.readline().strip()
            # EOF
            if len(line) == 0:
                break
            obj = json.loads(line.strip())
            statelock.acquire()
            updateworld(obj)
            statelock.release()

def poll_plan():
    while True:
        statelock.acquire()
        update_plan()
        statelock.release()
        sleep(3)

def shell():
    while True:
        try:
            line = raw_input('% ')
            if line == 'exit':
                break
            elif line == 'move-grab':
                planner.set_task('move-grab')
            elif line == 'turn-move-grab':
                planner.set_task('turn-move-grab')
            elif line == 'turn-shoot':
                planner.set_task('turn-shoot')
            elif line == 'left':
                statelock.acquire()
                world._our_side = 'left'
                world._their_side = 'right'
                statelock.release()
            elif line == 'right':
                statelock.acquire()
                world._our_side = 'right'
                world._their_side = 'left'
                statelock.release()
        except:
            break

player = argv[1]
comms.init(argv[2], '60', '+++')

mkfifo(visionpipe)
t = Thread(target=monitor_vision)
t.daemon = True
t2 = Thread(target=poll_plan)
t3 = Thread(target=shell)
t2.daemon = True
t.start()
t2.start()
t3.start()

p = subprocess.Popen(['../vision/vision', visionpipe], cwd='../vision')

t3.join()
rmtree(tmpdir)
