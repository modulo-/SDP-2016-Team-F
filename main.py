#!/usr/bin/python2

from Vision.vision import Vision
from planning.planner import AttackPlanner, DefencePlanner
from planning.comms import RFCommsManager, TractorCrabCommsManager
from planning.world import World
from threading import Timer, Thread
from sys import argv
import logging
from logging import debug, info, warning
from planning.predictor import Predictor
from getopt import getopt
from time import time

PITCH_NO = 0
color = None

class Interrupt:
    def __init__(self, cond, run, delay):
        self.last_t = 0
        self.cond = cond
        self.run = run
        self.delay = delay

predictor = Predictor()
latest_world = World('left', PITCH_NO)
latest_world.our_attacker._receiving_area = {'width': 40, 'height': 30, 'front_offset': 10}
latest_world.our_defender._receiving_area = {'width': 40, 'height': 30, 'front_offset': 10}
interrupts = []

def get_defender(world):
    if color == 'b':
        return world.robot_blue_green
    else:
        return world.robot_yellow_green

def get_attacker(world):
    if color == 'b':
        return world.robot_blue_pink
    else:
        return world.robot_yellow_pink

def get_green_opponent(world):
    if color == 'b':
        return world.robot_yellow_green
    else:
        return world.robot_blue_green

def get_pink_opponent(world):
    if color == 'b':
        return world.robot_yellow_pink
    else:
        return world.robot_blue_pink

def new_vision(world):
    predictor.update(world)
    world = predictor.predict()
    latest_world.update_positions(
        our_defender=get_defender(world),
        our_attacker=get_attacker(world),
        their_robot_0=get_green_opponent(world),
        their_robot_1=get_pink_opponent(world),
        ball=world.ball,
    )
    t = time()
    info('Hello')
    for interrupt in interrupts:
        if t - interrupt.last_t >= interrupt.delay and interrupt.cond():
            interrupt.last_t = t
            interrupt.run()
    if latest_world.our_defender.can_catch_ball(latest_world.ball):
        info('Can catch ball.')
    if latest_world.our_defender.is_missing():
        warning("Robot is missing!")


def start_vision():
    Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)

if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING, format="\r%(asctime)s - %(levelname)s - %(message)s")
    optlist, args = getopt(argv[1:], '1:2:',
        ['info', 'warn', 'error', 'debug'])
    attacker = None
    defender = None
    for (opt, arg) in optlist:
        if opt == '-1':
            defender = TractorCrabCommsManager(0, arg)
        elif opt == '-2':
            attacker = RFCommsManager(1, arg)
        elif opt == '--debug':
            logging.root.setLevel(logging.DEBUG)
        elif opt == '--info':
            logging.root.setLevel(logging.INFO)
        elif opt == '--warn':
            logging.root.setLevel(logging.WARNING)
        elif opt == '--error':
            logging.root.setLevel(logging.ERROR)
    if len(args) != 1 or args[0] not in ['b', 'blue', 'y', 'yellow']:
        print("Usage: ./main.py [-1PATH] [-2PATH] OPTIONS TEAM-COLOR")
        print("")
        print("Where -1 and -2 refer to group 11 and 12's RF devices.")
        print("TEAM-COLOR must be either 'blue' (or 'b') or 'yellow' (or 'y').")
        print("")
        print("Options:")
        print("")
        print("--debug Set logging level to 'debug'")
        print("--info  Set logging level to 'info'")
        print("--warn  Set logging level to 'warn'")
        print("--error Set logging level to 'error'")
        exit(0)
    if args[0] in ['b', 'blue']:
        color = 'b'
    else:
        color = 'y'
    print("")
    print("Enter a task into the shell to run it. Currently supported:")
    print(" - 'move-grab'")
    print(" - 'turn-shoot'")
    print("The following control commands are also available:")
    print(" - 'exit' to exit")
    print(" - 'debug' to set the logging level to debug")
    print(" - 'info' to set the logging level to info")
    print(" - 'warn' to set the logging level to warnings (default)")
    print(" - 'error' to set the logging level to errors")
    print("Enter 'exit' to exit.")
    thread = Thread(target=start_vision)
    thread.daemon = True
    thread.start()
    attack_planner = None
    defence_planner = None
    if attacker:
        attack_planner = AttackPlanner(comms=attacker)
        interrupts.append(Interrupt(
            lambda: latest_world.our_attacker.can_catch_ball(latest_world.ball),
            lambda: attack_planner.plan_and_act(latest_world), 2))
    if defender:
        defence_planner = DefencePlanner(comms=defender)
        interrupts.append(Interrupt(
            lambda: latest_world.our_defender.can_catch_ball(latest_world.ball),
            lambda: defender.close_grabbers(), 2))
            #lambda: defence_planner.plan_and_act(latest_world)))
    def run_planners():
        if attack_planner:
            attack_planner.plan_and_act(latest_world)
        if defence_planner:
            defence_planner.plan_and_act(latest_world)
        timer = Timer(1, run_planners)
        timer.daemon = True
        timer.start()

    timer = Timer(1, run_planners)
    timer.daemon = True
    timer.start()
    while True:
        task = None
        try:
            task = raw_input("").strip()
        except EOFError:
            exit(0)
        if task == 'exit':
            exit(0)
        elif task == 'debug':
            logging.root.setLevel(logging.DEBUG)
        elif task == 'info':
            logging.root.setLevel(logging.INFO)
        elif task == 'warn':
            logging.root.setLevel(logging.WARNING)
        elif task == 'error':
            logging.root.setLevel(logging.ERROR)
        else:
            # TODO: support seperate tasks for attacker and defender.
            if attack_planner:
                attack_planner.set_task(task)
            if defence_planner:
                defence_planner.set_task(task)
