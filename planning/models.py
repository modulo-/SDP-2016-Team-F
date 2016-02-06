import utils

# Base class for goals
class Goal (object):
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Return the next action necesary to achieve the goal
    def generate_action(self):
        raise NotImplemented

# Go to and grab ball
class GetBall (Goal):
    def generate_action(self):
        actions = [GrabBall(self, self.world, self.robot),
                   GoToStaticBall(world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
        return None

# Base class for actions
class Action (object):
    preconditions = []

    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Test the action's preconditions
    def is_possible(self):
        for condition in self.preconditions:
            if not condition(world):
                return False
        return True

    # Do comms to perform action
    def perform(self, comms):
        raise NotImplemented

class GoToStaticBall (Action):
    preconditions = [lambda w, r: utils.ball_is_static(w)]

    def perform(self, comms):
        comms.move_to(world.ball.x, world.ball.y)

class GrabBall (Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball)]

    def perform(self, comms):
        comms.close_grabbers()
