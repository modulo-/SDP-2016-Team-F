from comms import CommsManager
from models import GetBall, Score, GrabBall, Shoot


class Planner:
    '''
    Takes the world and creates a plan and actuates it.
    '''

    def set_task(self, task):
        self.current_task = task

    def __init__(self, comms=CommsManager(0)):
        self.comms = comms
        self.previous_action = None
        self.current_task = None
        self.grabber_state = 'OPEN'

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
        action.perform(self.comms)
        if isinstance(action, GrabBall):
            self.grabber_state = 'CLOSED'
        elif isinstance(action, Shoot):
            self.grabber_state = 'OPEN'

    def plan_and_act(self, world):
        '''
        Make plans for each robot and perform them
        '''
        robot = world.our_robot
        world.our_robot.catcher = self.grabber_state
        goal = self.get_goal(world, robot)
        if goal == None:
            return
        action = goal.generate_action()
        if action != self.previous_action:
            action = action
        self.previous_action = action
        self.actuate(action)
