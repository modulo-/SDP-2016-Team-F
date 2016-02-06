import utils

class Goal (object):
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    def generate_action(self):
        raise NotImplemented

class GetBall (Goal):
    def generate_action(self):
        actions = [CatchBall(self, self.world, self.robot),
                   GoToStaticBall(world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
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
    preconditions = [lambda w, r: utils.ball_is_static(w)]

    def perform(self, comms):
        comms.move_to(world.ball.x, world.ball.y)

class CatchBall (Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball)]

    def perform(self, comms):
        comms.close_grabbers()
