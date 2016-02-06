from comms import CommsManager
from models import *

class Planner:

    def __init__(self):
        self.comms = [CommsManager(0)]

    # Get goal for robot in world
    def get_goal(self, world, robot):
        raise NotImplemented

    # Perform actions
    def actuate(self, *actions):
        for a in actions:
            a.perform(self.comms)

    # Make plans for each robot and perform them
    def plan_and_act(self, world):
        # For now plan for just one robot
        robot = world.our_attacker()
        goal = get_goal(world, robot)
        action = goal.generate_action()
        messages = action.get_messages()
        actuate(action)
