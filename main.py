#!/usr/bin/python2

from Vision.vision import Vision
from planning.planner import AttackPlanner, DefencePlanner
from planning.comms import RFCommsManager, TractorCrabCommsManager
from planning.world import World
from threading import Timer, Thread
from sys import argv
import logging
from logging import debug, warning
from getopt import getopt


PITCH_NO = 0
color = None

latest_world = World('left', PITCH_NO)
latest_world.our_attacker._receiving_area = {'width': 40, 'height': 50, 'front_offset': 20}
latest_world.our_defender._receiving_area = {'width': 40, 'height': 50, 'front_offset': 20}

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

def new_vision(world):
    latest_world.update_positions(
        our_defender=get_defender(world),
        our_attacker=get_attacker(world),
        ball=world.ball,
    )
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
    if defender:
        defence_planner = DefencePlanner(comms=defender)
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
