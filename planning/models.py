import utils

# Base class for goals
class Goal (object):
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Return the next action necesary to achieve the goal
    def generate_action(self):
        raise NotImplementedError

# Go to and grab ball
class GetBall (Goal):
    def generate_action(self):
        actions = [GrabBall(self.world, self.robot),
                   GoToStaticBall(self.world, self.robot),
                   TurnToBall(self.world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
        return None

class Score (Goal):
    def generate_action(self):
        actions = [Shoot(self.world, self.robot),
                   TurnToGoal(self.world, self.robot)]
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
            if not condition(self.world, self.robot):
                return False
        return True

    # Do comms to perform action
    def perform(self, comms):
        raise NotImplementedError

    # Get messages relating to action
    def get_messages(self):
        return []

class GoToStaticBall (Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: r.get_rotation_to_point(w.ball.x, w.ball.y) == 0]

    def perform(self, comms):
        comms.move_to(self.world.ball.x, self.world.ball.y)

class GrabBall (Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball)]

    def perform(self, comms):
        comms.close_grabbers()

class TurnToGoal (Action):
    preconditions = [lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        # TODO find best point to shoot to
        x = self.world.goal.x + self.world.goal.width / 2
        y = self.world.goal.y
        comms.turn(self.robot.get_rotation_to_point(x, y))

class TurnToBall (Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(self.robot.get_rotation_to_point(x, y))

class Shoot (Action):
    preconditions = [lambda w, r: r.has_ball(w.ball),
                     lambda w, r: utils.can_score(w, r, w.their_goal())]

    def perform(self, comms):
        comms.kick_full_power()
