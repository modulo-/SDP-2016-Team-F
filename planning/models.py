import utils
import math

# TODO
ROTATION_THRESHOLD = 0.35


class Goal(object):
    '''
    Base class for goals
    '''
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Return the next action necesary to achieve the goal
    def generate_action(self):
        raise NotImplementedError


class GetBall(Goal):
    '''
    Go to and grab ball
    '''
    def generate_action(self):
        actions = [GrabBall(self.world, self.robot),
                   GoToStaticBall(self.world, self.robot),
                   TurnToBall(self.world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
        return None


class Score(Goal):
    def generate_action(self):
        actions = [Shoot(self.world, self.robot),
                   TurnToGoal(self.world, self.robot)]
        for a in actions:
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


class GoToStaticBall(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(r.get_rotation_to_point(w.ball.x, w.ball.y)) < ROTATION_THRESHOLD]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 10
        proportion = (d - grabber_size) / d
        target_pos = (self.robot.x + dx * proportion, self.robot.y + dy * proportion, self.robot.angle)
        comms.move(robot_pos=(self.robot.x, self.robot.y, self.robot.angle), target_pos=target_pos, distance=(d - grabber_size))


class GrabBall(Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball)]

    def perform(self, comms):
        comms.close_grabbers()


class TurnToGoal(Action):
    #preconditions = [lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        # TODO find best point to shoot to
        #x = self.world.goal.x + self.world.goal.width / 2
        #y = self.world.goal.y
        x = 0
        y = 100
        comms.turn(self.robot.get_rotation_to_point(x, y))


class TurnToBall(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(self.robot.get_rotation_to_point(x, y))


class Shoot(Action):
    #preconditions = [lambda w, r: r.has_ball(w.ball),
    #                 lambda w, r: utils.can_score(w, r, w.their_goal())]

    def perform(self, comms):
        comms.kick_full_power()
