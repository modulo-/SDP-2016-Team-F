import models_attacker as attacker
import models_defender as defender

from comms import CommsManager
from logging import info


class Planner (object):
    '''
    Takes the world and creates a plan and actuates it.
    '''

    def __init__(self, comms=CommsManager(0)):
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
        raise NotImplementedError

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


class AttackPlanner(Planner):
    '''
    Planner for attacking robot
    '''

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''
        if robot.has_ball(world.ball):
            return attacker.Score(world, robot)
        else:
            return attacker.GetBall(world, robot)
"""        if self.current_task == 'move-grab':
            return attacker.GetBall(world, robot)
        elif self.current_task == 'turn-move-grab':
            return attacker.GetBall(world, robot)
        elif self.current_task == 'turn-shoot':
            return attacker.Score(world, robot)"""


class DefencePlanner(Planner):
    '''
    Planner for defending robot
    '''

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''
        if self.current_task == 'move-grab':
            return defender.GetBall(world, robot)
        if self.current_task == 'm31':
            return defender.ReceivingPass(world, robot)
