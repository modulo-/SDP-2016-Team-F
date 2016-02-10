
from Vision.vision import Vision
from planning.planner import Planner
from planning.comms import CommsManager, RFCommsManager
from planning.world import World
from threading import Timer, Thread

task = raw_input("Enter task (move-grab or turn-shoot): ")

PITCH_NO = 0

latest_world = World('left', PITCH_NO)
latest_world.our_robot._receiving_area = {'width': 40, 'height': 50, 'front_offset': 20}


def new_vision(world):
    latest_world.update_positions(
        {
            "our_robot": world.robot_blue_green,
            "ball": world.ball,
        }
    )
    if latest_world.our_robot.is_missing():
        print ("Robot is missing!")
    
def start_vision():
	vision = Vision(video_port=0, pitch=PITCH_NO, planner_callback=new_vision)
	
thread = Thread(target=start_vision)
thread.start()
comms = RFCommsManager(0, "/dev/ttyACM0")
planner = Planner(comms=comms)
planner.set_task(task)

def run_planner():
    print (latest_world.our_robot.angle)
    planner.plan_and_act(latest_world)
    timer = Timer(1, run_planner)
    timer.start()
    
timer = Timer(1, run_planner)
timer.start()
