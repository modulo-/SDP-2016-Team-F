from comms import CommsManager
from models import *

class Planner:

    def set_task(self, task):
        self.current_task = task

    def __init__(self):
        self.comms = [CommsManager(0)]
        self.robot_actions = [None]
        self.current_task = 'move-grab'

    # Get goal for robot in world
    def get_goal(self, world, robot):
        # For milestone 2
        if self.current_task == 'move-grab':
            return GetBall(world, robot)
        elif self.current_task == 'turn-move-grab':
            return GetBall(world, robot)
        elif self.current_task == 'turn-shoot':
            return Score(world, robot)

    # Perform actions
    def actuate(self, actions):
        for i in range(len(actions)):
            actions[i].perform(self.comms[i])

    # Make plans for each robot and perform them
    def plan_and_act(self, world):
        # For now plan for just one robot
        robot = world.our_attacker
        goal = self.get_goal(world, robot)
        action = goal.generate_action()
        messages = action.get_messages()
        actions = [None]
        if action != self.robot_actions[0]:
            actions[0] = action
        self.robot_actions = actions
        self.actuate(actions)
