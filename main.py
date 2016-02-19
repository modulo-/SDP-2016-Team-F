#!/usr/bin/python2

from Vision.vision import Vision
from planning.planner import Planner
from planning.comms import CommsManager, RFCommsManager, TractorCrabCommsManager
from planning.world import World
from threading import Timer, Thread
from sys import argv
import readline
import logging
from logging import debug, warning


PITCH_NO = 0

latest_world = World('left', PITCH_NO)
latest_world.our_defender._receiving_area = {'width': 40, 'height': 50, 'front_offset': 20}


def new_vision(world):
    latest_world.update_positions(
        {
            "our_defender": world.robot_blue_green,
            "ball": world.ball,
        }
    )
    if latest_world.our_defender.is_missing():
        warning("Robot is missing!")
    
def start_vision():
	vision = Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)
	
if __name__ == '__main__':
    logging.basicConfig(level=logging.WARNING, format=
        "\r%(asctime)s - %(levelname)s - %(message)s")
    if len(argv) != 3:
        print("Usage: ./main.py <group> <rf device path>")
        print("<group> must be either '11' or '12'.")
        print("<rf device path> should be '/dev/ttyACM0' or similar.")
        print("Note that I'm lazy and don't check the input properly.")
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
        exit(0)
    thread = Thread(target=start_vision)
    thread.daemon = True
    thread.start()
    comms = None
    if argv[1] == '11':
        comms = TractorCrabCommsManager(0, argv[2])
    elif argv[1] == '12':
        comms = RFCommsManager(0, argv[2])
    else:
        raise Exception('You wish.')
    planner = Planner(comms=comms)

    def run_planner():
        debug(latest_world.our_defender.angle)
        planner.plan_and_act(latest_world)
        timer = Timer(1, run_planner)
        timer.daemon = True
        timer.start()
        
    timer = Timer(1, run_planner)
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
            planner.set_task(task)
