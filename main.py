#!/usr/bin/python2

from Vision.vision import Vision
from planning.planner import Planner
from planning.comms import CommsManager, RFCommsManager, TractorCrabCommsManager
from planning.world import World
from threading import Timer, Thread
from sys import argv


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
        print ("Robot is missing!")
    
def start_vision():
	vision = Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)
	
if __name__ == '__main__':
    if len(argv) != 3:
        print("Usage: ./main.py <group> <rf device path>")
        print("<group> must be either '11' or '12'.")
        print("<rf device path> should be '/dev/ttyACM0' or similar.")
        print("Note that I'm lazy and don't check the input properly.")
        exit(0)
    task = raw_input("Enter task ('move-grab' or 'turn-shoot'): ")
    thread = Thread(target=start_vision)
    thread.start()
    comms = None
    if argv[1] == '11':
        comms = TractorCrabCommsManager(0, argv[2])
    elif argv[1] == '12':
        comms = RFCommsManager(0, argv[2])
    else:
        raise Exception('You wish.')
    planner = Planner(comms=comms)
    planner.set_task(task.strip())

    def run_planner():
        print (latest_world.our_defender.angle)
        planner.plan_and_act(latest_world)
        timer = Timer(1, run_planner)
        timer.start()
        
    timer = Timer(1, run_planner)
    timer.start()
