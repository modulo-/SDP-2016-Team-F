import math
from logging import info

ROTATION_THRESHOLD = 0.1
FACING_ROTATION_THRESHOLD = 0.4
DISTANCE_THRESHOLD = 5

DEFAULT_DELAY = 4


def are_equivalent_positions(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2) < DISTANCE_THRESHOLD


class Goal(object):
    '''
    Base class for goals
    '''
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Return the next action necesary to achieve the goal
    def generate_action(self):
        info("Generating action for goal: {0}".format(self.__class__.__name__))
        for a in self.actions:
            if a.is_possible():
                return a
        return None


class Action(object):
    '''
    Base class for actions
    '''
    preconditions = []

    def __init__(self, world, robot, additional_preconds=[]):
        self.world = world
        self.robot = robot
        self.preconditions = self.__class__.preconditions + additional_preconds

    # Test the action's preconditions
    def is_possible(self):
        info("Testing action : {0}".format(self.__class__.__name__))
        for (condition, name) in self.preconditions:
            if not condition(self.world, self.robot):
                info("Precondition is false: {0}".format(name))
                return False
        info("Action possible: {0}".format(self.__class__.__name__))
        return True

    # Do comms to perform action
    def perform(self, comms):
        raise NotImplementedError

    # Get messages relating to action
    def get_messages(self):
        return []

    def get_delay(self):
        return DEFAULT_DELAY
