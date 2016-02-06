import utils

class Goal (object):
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    def generate_action(self):
        raise NotImplemented

class GetBall (Goal):
    def generate_action(self):
        goto_static = GoToStaticBall(world, self.robot)
        if goto_static.is_possible():
            return goto_static
        else:
            return None

class Action (object):
    preconditions = []

    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    def is_possible(self):
        for condition in self.preconditions:
            if not condition(world):
                return False
        return True

    def perform(self, comms):
        raise NotImplemented

class GoToStaticBall (Action):
    preconditions = [utils.ball_is_static]

    def perform(self, comms):
        comms.move_to(world.ball.x, world.ball.y)
