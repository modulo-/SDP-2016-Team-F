import math

# TODO
ROTATION_THRESHOLD = 0.35
FACING_ROTATION_THRESHOLD = 0.35
DISTANCE_THRESHOLD = 5


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
        for a in self.actions:
            if a.is_possible():
                return a
        return None


class Action(object):
    '''
    Base class for actions
    '''
    preconditions = []

    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Test the action's preconditions
    def is_possible(self):
        for condition in self.preconditions:
            if not condition(self.world, self.robot):
                return False
        return True

    # Do comms to perform action
    def perform(self, comms):
        raise NotImplementedError

    # Get messages relating to action
    def get_messages(self):
        return []
