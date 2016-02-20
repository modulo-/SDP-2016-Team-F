
from comms import CommsManager
import models_attacker as attacker
import models_defender as defender
from logging import info


class Planner:
    '''
    Takes the world and creates a plan and actuates it.
    '''

    def __init__(self, planner_type, comms=CommsManager(0)):
        if planner_type is None:
            raise Exception("Planner type has to be defined! (use 11 or 12)")

        self.planner_type = planner_type
        self.comms = comms
        self.previous_action = None
        self.current_task = 'move-grab'
        self.grabber_state = 'CLOSED'

    def set_task(self, task):
        self.current_task = task

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''

        if self.planner_type == "11":
            if self.current_task == 'move-grab':
                return defender.GetBall(world, robot)
        elif self.planner_type == "12":
            if self.current_task == 'move-grab':
                return attacker.GetBall(world, robot)
            elif self.current_task == 'turn-move-grab':
                return attacker.GetBall(world, robot)
            elif self.current_task == 'turn-shoot':
                return attacker.Score(world, robot)
        else:
            raise Exception("Wrong planner type (use 11 or 12)")

    def actuate(self, action):
        '''Perform actions'''
        action.perform(self.comms)
        if isinstance(action, defender.GrabBall) or isinstance(action, attacker.GrabBall):
            info("Did grab")
            self.grabber_state = 'CLOSED'
        elif isinstance(action, attacker.Shoot) or isinstance(action, attacker.OpenGrabbers):
            info("Did open")
            self.grabber_state = 'OPEN'

    def plan_and_act(self, world):
        '''
        Make plans for each robot and perform them
        '''
        world.our_defender.catcher = self.grabber_state
        robot = world.our_defender
        goal = self.get_goal(world, robot)
        if goal is None:
            return
        action = goal.generate_action()
        if action != self.previous_action:
            action = action
        self.previous_action = action
        self.actuate(action)
