from Vision.vision import Vision
from planning.planner import Planner
from planning.comms import CommsManager
from planning.world import World
from threading import Timer

PITCH_NO = 0

latest_world = World('left', PITCH_NO)

def new_vision(world):
    latest_world.update_positions(
        {
            "our_robot": world.robot_blue_pink,
            "ball": world.ball
        }
    )

vision = Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)
comms = RFCommsManager(0, "/dev/ttyACM[NUMBER]")
planner = Planner(comms=comms)

def run_planner():
    planner.plan_and_act(latest_world)
    timer.start()
    
timer = Timer(1, run_planner)
timer.start()
