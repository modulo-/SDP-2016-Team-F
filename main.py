#!/usr/bin/python2

import getopt
import logging
import sys

from Vision.vision import Vision
from planning.planner import AttackPlanner, DefencePlanner
from planning.comms import RFCommsManager, TractorCrabCommsManager
from planning.world import World
from threading import Timer, Thread
from planning.predictor import Predictor
from time import time
from math import pi
from planning import utils

color = None


class Interrupt:
    def __init__(self, cond, run, delay):
        self.last_t = 0
        self.cond = cond
        self.run = run
        self.delay = delay

INITIAL_PLANNER_DELAY = 4

predictor = Predictor()
# TODO: formally, the 0 should be the command-line set pitch number.
# But, since it isn't used, it doesn't really matter.
latest_world = World('left', 0)
latest_world.our_attacker._receiving_area = {'width': 40, 'height': 20, 'front_offset': 15}#{'width': 25, 'height': 10, 'front_offset': 20}
latest_world.our_defender._receiving_area = {'width': 50, 'height': 35, 'front_offset': 0}
interrupts = []


def get_attacker(world):
    if color == 'b':
        return world.robot_blue_pink
    else:
        return world.robot_yellow_pink


def get_defender(world):
    if color == 'b':
        return world.robot_blue_green
    else:
        return world.robot_yellow_green


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

def new_grabber_state(value):
    self.world.is_ball_in_grabbers = value

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
    for interrupt in interrupts:
        if t - interrupt.last_t >= interrupt.delay and interrupt.cond():
            interrupt.last_t = t
            interrupt.run()
    if latest_world.game_state in ['kickoff-them', 'kickoff-us', 'penalty-defend',
            'penalty-shoot'] and not utils.ball_is_static(latest_world):
        latest_world.game_state = 'normal-play'


def start_vision(pitch_no):
    Vision(video_port=0, pitch=pitch_no, planner_callback=new_vision)


def help():
    print("Usage: ./main.py --plan plan --pitch {0|1} --goal {left|right} [--defender PATH | --attacker PATH] [--logging level] [--color color]")
    print("")
    print("Where --defender(-1) and --attacker(-2) refer to group 11 and 12's RF devices.")
    print("")
    print("--logging (-l) options:")
    print("\t--debug Set logging level to 'debug'")
    print("\t--info  Set logging level to 'info'")
    print("\t--warn  Set logging level to 'warn'")
    print("\t--error Set logging level to 'error'")
    print("")
    print("--color (-c) options:")
    print("\t'blue' (or 'b')")
    print("\t'yellow' (or 'y')")
    print("")
    exit(0)


def usage():
    print("")
    print("Enter a task into the shell to run it. Currently supported:")
    print(" - 'move-grab'")
    print("The following control commands are also available:")
    print(" - 'exit' to exit")
    print(" - 'debug' to set the logging level to debug")
    print(" - 'info' to set the logging level to info")
    print(" - 'warn' to set the logging level to warnings (default)")
    print(" - 'error' to set the logging level to errors")
    print("Enter 'exit' to exit.")


def set_logging(mode):
    if mode == 'debug':
        logging.root.setLevel(logging.DEBUG)
    elif mode == 'info':
        logging.root.setLevel(logging.INFO)
    elif mode == 'warn':
        logging.root.setLevel(logging.WARNING)
    elif mode == 'error':
        logging.root.setLevel(logging.ERROR)


def set_plan(attack_planner, defence_planner, plan):
    logging.info('>>> Setting the initial plan to: ' + str(plan))
    # TODO: support seperate tasks for attacker and defender.
    if attack_planner:
        attack_planner.set_task(plan)
    if defence_planner:
        defence_planner.set_task(plan)


def main():
    global color
    pitch_no = 0
    logging.basicConfig(level=logging.WARNING, format="\r%(asctime)s - %(levelname)s - %(message)s")
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hz1:2:l:c:p:g:", ["help",
            "visible", "defender=", "attacker=", "logging=", "color=", "plan=",
            "goal=", "pitch="])
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err)  # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    attacker = None
    defender = None
    plan = None
    for o, a in opts:
        if o in ("-h", "--help"):
            help()
            sys.exit()
        elif o in ("-z", "--visible"):
            logger = logging.getLogger()
            formatter = logging.Formatter("\033[95m\r%(asctime)s - %(levelname)s - %(message)s\033[0m")
            logger.handlers[0].setFormatter(formatter)
        elif o in ("-1", "--defender"):
            defender = TractorCrabCommsManager(0, a)
        elif o in ("-2", "--attacker"):
            attacker = RFCommsManager(0, a, new_grabber_state)
        elif o in ("-l", "--logging"):
            logging_modes = a.split(",")
            for mode in logging_modes:
                set_logging(mode)
        elif o in ("-c", "--color"):
            if a == 'b' or a == 'blue':
                color = 'b'
            elif a == 'y' or a == 'yellow':
                color = 'y'
            else:
                print "Unsupported color. Default is 'blue'"
                color = 'b'
        elif o in ("-p", "--plan"):
            plan = a
        elif o in ("-g", "--goal"):
            latest_world.our_side = a
        elif o == "--pitch":
            pitch_no = int(a)
        else:
            assert False, "unhandled option"
            exit(0)

    usage()
    run(attacker=attacker, defender=defender, plan=plan, pitch_no=pitch_no)


def run(attacker, defender, plan, pitch_no):
    thread = Thread(target=start_vision, args=(pitch_no,))
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
            lambda: defence_planner.plan_and_act(latest_world), 2))

    def run_planners():
        delay = None
        if attack_planner:
            delay = attack_planner.plan_and_act(latest_world)
        if defence_planner:
            delay = defence_planner.plan_and_act(latest_world)
        timer = Timer(delay, run_planners)
        timer.daemon = True
        timer.start()

    timer = Timer(INITIAL_PLANNER_DELAY, run_planners)
    timer.daemon = True
    timer.start()

    # set initial plan
    set_plan(attack_planner, defence_planner, plan)

    while True:
        task = None
        try:
            task = raw_input("").strip()
        except EOFError:
            exit(0)
        if task == 'exit':
            exit(0)
        elif task in ['debug', 'info', 'warn', 'error']:
            set_logging(task)
        elif task == 'switch':
            if latest_world.our_side == 'left':
                latest_world.our_side = 'right'
            else:
                latest_world.our_side = 'left'
        elif task in ['kickoff-them', 'kickoff-us', 'normal-play', 'penalty-defend',
                'penalty-shoot']:
            latest_world.game_state = task
        elif task == 'game-stop':
            latest_world.game_state = None
        elif task == 'penalty11':
            latest_world.our_defender.penalty = True
        elif task == 'unpenalty11':
            latest_world.our_defender.penalty = False
        else:
            set_plan(attack_planner, defence_planner, task)

if __name__ == "__main__":
    main()
