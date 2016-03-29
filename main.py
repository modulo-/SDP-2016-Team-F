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
from Tkinter import Tk, Button, Label, StringVar

color = None
statev = None


class Interrupt:
    def __init__(self, cond, run, delay):
        self.last_t = 0
        self.cond = cond
        self.run = run
        self.delay = delay

INITIAL_PLANNER_DELAY = 4

attacker_grabber_close = None
ATTACKER_GRABBERS_CLOSE_TIME = 4
attacker_grabbers_close_timer = None

predictor = Predictor()
# TODO: formally, the 0 should be the command-line set pitch number.
# But, since it isn't used, it doesn't really matter.
latest_world = World('left', 0)
latest_world.our_attacker._receiving_area = {'width': 40, 'height': 20, 'front_offset': 15}#{'width': 25, 'height': 10, 'front_offset': 20}
latest_world.our_defender._receiving_area = {'width': 50, 'height': 35, 'front_offset': 0}
interrupts = []
attack_timer = None
defence_timer = None

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
    global attacker_grabbers_close_timer
    if value == "grabbersOpen":
        print ("Got grabbersOpen")
        latest_world.our_attacker.is_ball_in_grabbers = False
        latest_world.our_attacker.catcher = "OPEN"
        try:
            attacker_grabbers_close_timer.start()
        except RuntimeError:
            # Timer already running, shouldn't happen
            pass
    else:
        if value == "NC":
            latest_world.our_attacker.is_ball_in_grabbers = False
        elif value == "BC":
            latest_world.our_attacker.is_ball_in_grabbers = True
        latest_world.our_attacker.catcher = "CLOSED"
        attacker_grabbers_close_timer.cancel()
        attacker_grabbers_close_timer = Timer(ATTACKER_GRABBERS_CLOSE_TIME,
                                              attacker_grabbers_close)


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
        statev.set('normal-play')


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
    pitch_no = 1
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

def do_ui():
    global statev
    top = Tk()
    def setstate(s):
        statev.set(s)
        if s == "game-stop":
            s = None
        latest_world.game_state = s
    statev = StringVar(top, 'game-stop')
    ko_us = Button(top, text="kickoff-us", command=lambda:setstate("kickoff-us"))
    ko_them = Button(top, text="kickoff-them", command=lambda:setstate("kickoff-them"))
    pn_def = Button(top, text="penalty-defend", command=lambda:setstate("penalty-defend"))
    pn_sht = Button(top, text="penalty-shoot", command=lambda:setstate("penalty-shoot"))
    nm_play = Button(top, text="normal-play", command=lambda:setstate("normal-play"))
    stp = Button(top, text="game-stop", command=lambda:setstate("game-stop"))
    statel = Label(top, textvariable=statev)
    ko_us.pack()
    ko_them.pack()
    pn_def.pack()
    pn_sht.pack()
    nm_play.pack()
    stp.pack()
    statel.pack()
    top.mainloop()

def run(attacker, defender, plan, pitch_no):
    global attack_timer, defence_timer, attacker_grabbers_close_timer,\
        attacker_grabbers_close
    ui_thread = Thread(target=do_ui)
    ui_thread.daemon = True
    ui_thread.start()

    vision_thread = Thread(target=start_vision, args=(pitch_no,))
    vision_thread.daemon = True
    vision_thread.start()

    attack_planner = None
    defence_planner = None

    def run_attack_planner():
        global attack_timer
        delay = attack_planner.plan_and_act(latest_world)
        attack_timer.cancel()
        attack_timer = Timer(delay, run_attack_planner)
        attack_timer.daemon = True
        attack_timer.start()

    def run_defence_planner():
        global defence_timer
        delay = defence_planner.plan_and_act(latest_world)
        defence_timer.cancel()
        defence_timer = Timer(delay, run_defence_planner)
        defence_timer.daemon = True
        defence_timer.start()

    def close_attacker_grabbers():
        print("Time out - closing grabbers")
        attacker.close_grabbers()
        attacker_grabbers_close_timer = Timer(ATTACKER_GRABBERS_CLOSE_TIME,
                                              close_attacker_grabbers)
    attacker_grabbers_close = close_attacker_grabbers

    if attacker:
        attack_planner = AttackPlanner(comms=attacker)
        """interrupts.append(Interrupt(
            lambda: latest_world.our_attacker.can_catch_ball(latest_world.ball),
            run_attack_planner, 2))"""
        attacker_grabbers_close_timer = Timer(ATTACKER_GRABBERS_CLOSE_TIME,
                                              close_attacker_grabbers)
    if defender:
        defence_planner = DefencePlanner(comms=defender)
        interrupts.append(Interrupt(
            lambda: latest_world.our_defender.can_catch_ball(latest_world.ball),
            run_defence_planner, 2))
        interrupts.append(Interrupt(
            lambda: utils.ball_heading_to_our_goal(latest_world) and latest_world.in_our_half(latest_world.ball),
            run_defence_planner, 2))

    if attack_planner:
        attack_timer = Timer(INITIAL_PLANNER_DELAY, run_attack_planner)
        attack_timer.daemon = True
        attack_timer.start()
    if defence_planner:
        defence_timer = Timer(INITIAL_PLANNER_DELAY, run_defence_planner)
        defence_timer.daemon = True
        defence_timer.start()


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
        elif task == 'penalty11':
            latest_world.our_defender.penalty = True
        elif task == 'unpenalty11':
            latest_world.our_defender.penalty = False
        else:
            set_plan(attack_planner, defence_planner, task)

if __name__ == "__main__":
    main()
