from comms import CommsManager
from models import GetBall, Score


class Planner:
    '''
    Takes the world and creates a plan and actuates it.
    '''

    def set_task(self, task):
        self.current_task = task

    def __init__(self):
        self.comms = [CommsManager(0)]
        self.previous_action = None
        self.current_task = 'move-grab'

    def get_goal(self, world, robot):
        '''
        State machine for the milestone 2
        '''
        if self.current_task == 'move-grab':
            return GetBall(world, robot)
        elif self.current_task == 'turn-move-grab':
            return GetBall(world, robot)
        elif self.current_task == 'turn-shoot':
            return Score(world, robot)

    def actuate(self, action):
        '''Perform actions'''
        action.perform(self.comms[0])

    def plan_and_act(self, world):
        '''
        Make plans for each robot and perform them
        '''
        robot = world.our_robot
        goal = self.get_goal(world, robot)
        action = goal.generate_action()
        if action != self.previous_action:
            action = action
        self.previous_action = action
        self.actuate(action)
