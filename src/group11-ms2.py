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
import planning
from math import pi
import math
import json
import subprocess
import readline
import comms
import group11cmd

def translate_pos(p):
    return p / 2.31

class CommsManager(planning.comms.CommsManager):
    def __init__(self):
        self.robot_index = 11

    def move(self, robot_pos, target_pos, distance):
        print("Moving robot {0} from {1} to {2} with distance {3}".format(self.robot_index, robot_pos, target_pos, distance))
        group11cmd.run(group11cmd.cmd_mv(
            group11cmd.parse_dist(robot_pos[0]),
            group11cmd.parse_dist(robot_pos[1]),
            group11cmd.parse_angle((-robot_pos[2] / pi * 180 + 90) % 360),
            group11cmd.parse_dist(target_pos[0]),
            group11cmd.parse_dist(target_pos[1]),
            group11cmd.parse_angle((-target_pos[2] / pi * 180 + 90) % 360)))
    
    def turn(self, angle):
        print("Turning robot {0} angle {1} radians ( {2} degrees)".format(self.robot_index, angle, math.degrees(angle)))
        group11cmd.run(group11cmd.cmd_spin(
            group11cmd.parse_angle(-angle / pi * 180)))
    
    def kick(self, distance):
        print("Robot {0} kicking distance {1}".format(self.robot_index, distance))
        group11cmd.run(group11cmd.cmd_kick(100))
    
    def kick_full_power(self):
        print("Robot {0} kicking full power".format(self.robot_index))
        group11cmd.run(group11cmd.cmd_kick(100))
    
    def close_grabbers(self):
        print("Robot {0} closing grabbers".format(self.robot_index))
        group11cmd.run(group11cmd.cmd_grabber_close() + group11cmd.cmd_wait(1000) + group11cmd.cmd_grabber_open())
    
    def release_grabbers(self):
        print("Robot {0} releasing grabbers".format(self.robot_index))
        group11cmd.run(group11cmd.cmd_grabber_open())

planner = planner.Planner(CommsManager())
tmpdir = mkdtemp()
visionpipe = path.join(tmpdir, 'visionpipe')

# TODO: better way to initialize.
world = world.World('left', 0)
statelock = Lock()

def update_plan():
    #group11cmd.run(group11cmd.cmd_brake(100) + group11cmd.cmd_mv(
    #        group11cmd.parse_dist(world.our_robot.x),
    #        group11cmd.parse_dist(world.our_robot.y),
    #        group11cmd.parse_angle(world.our_robot.angle / pi * 180),
    #        group11cmd.parse_dist(world.ball.x),
    #        group11cmd.parse_dist(world.ball.y),
    #        group11cmd.parse_angle(0)))
    planner.plan_and_act(world)
    pass

def updateworld(obj):
    robvec = world.our_robot.vector
    ballvec = world.ball.vector
    if player in obj:
        robvec = Vector(
            translate_pos(obj[player]['x']),
            200 - translate_pos(obj[player]['y']),
            (-(obj[player]['f'] * pi / 180 - pi / 2)) % (2 * pi), 0)
    if 'b' in obj:
        ballvec = Vector(
            translate_pos(obj['b']['x']),
            200 - translate_pos(obj['b']['y']), 0, 0)
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
            try:
                obj = json.loads(line.strip())
                statelock.acquire()
                updateworld(obj)
                statelock.release()
            except:
                pass

def poll_plan():
    while True:
        sleep(6)
        statelock.acquire()
        update_plan()
        statelock.release()

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
            elif line == 'stop':
                planner.set_task(None)
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
comms.init(argv[2], '60', '~~~')

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
