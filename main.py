from Vision.vision import Vision
from planning.planner import Planner
from planning.comms import CommsManager
from planning.world import World
from threading import Timer, Thread

PITCH_NO = 0

latest_world = World('left', PITCH_NO)
latest_world.our_robot._receiving_area = {'width': 30, 'height': 30, 'front_offset': 12}


def new_vision(world):
    latest_world.update_positions(
        {
            "our_robot": world.robot_blue_pink,
            "ball": world.ball
        }
    )
    
def start_vision():
	vision = Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)
	
thread = Thread(target=start_vision)
thread.start()
comms = CommsManager(0)
planner = Planner(comms=comms)

def run_planner():
    planner.plan_and_act(latest_world)
    timer = Timer(1, run_planner)
    timer.start()
    
timer = Timer(1, run_planner)
timer.start()
