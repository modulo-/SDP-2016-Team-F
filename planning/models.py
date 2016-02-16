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
                   OpenGrabbers(self.world, self.robot),
                   GoToOpeningDistanceStaticBall(self.world, self.robot),
                   TurnToBall(self.world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
        return None


class Score(Goal):
    '''
    Turn and shoot
    '''
    def generate_action(self):
        actions = [Shoot(self.world, self.robot),
                   TurnToGoal(self.world, self.robot)]
        for a in actions:
            if a.is_possible():
                return a
        return None

class DefendGoal(Goal):
    '''
    Move around goal to block attacker
    '''
    def generate_action(self):
        raise NotImplementedError

class Pass(Goal):
    '''
    Pass to attacker
    '''
    def generate_action(self):
        raise NotImplementedError

class Tactical(Goal):
    '''
    Move to optimum defensive position
    '''
    def generate_action(self):
        raise NotImplementedError

class Position(Goal):
    '''
    Move to optimum attacking position
    '''
    def generate_action(self):
        raise NotImplementedError

class Block(Goal):
    '''
    Our attacker block attacking opponent
    '''
    def generate_action(self):
        raise NotImplementedError


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
    preconditions = [#lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(r.get_rotation_to_point(w.ball.x, w.ball.y)) < ROTATION_THRESHOLD,
        lambda w, r: r.catcher == 'OPEN']

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 24
        comms.move(d - grabber_size)

class GoToOpeningDistanceStaticBall(Action):
    preconditions = [#lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(r.get_rotation_to_point(w.ball.x, w.ball.y)) < ROTATION_THRESHOLD]
        #lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) > 60]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 70
        comms.move(d - grabber_size)

class OpenGrabbers(Action):
    preconditions = [lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) < 80,
                     lambda w, r: r.catcher == 'CLOSED']

    def perform(self, comms):
        comms.release_grabbers()

class GrabBall(Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball),
                     lambda w, r: r.catcher == 'OPEN']

    def perform(self, comms):
        comms.close_grabbers()


class TurnToGoal(Action):
    preconditions = []#lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        # TODO find best point to shoot to
        #x = self.world.their_goal.x + self.world.their_goal.width / 2
        #x = (self.world.their_goal.higher_post -
        #     self.world.their_goal.lower_post) / 2
        x = 220
        y = 0 #self.world.their_goal.y
        comms.turn(self.robot.get_rotation_to_point(x, y))

class TurnToBall(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(self.robot.get_rotation_to_point(x, y))


class Shoot(Action):
    #preconditions = [lambda w, r: abs(r.get_rotation_to_point((w.their_goal.higher_post - w.their_goal.lower_post) / 2, 0)) < 0.35/(1+r.get_displacement_to_point((w.their_goal.higher_post - w.their_goal.lower_post) / 2, 0))]
    #preconditions = [lambda w, r: abs(r.get_rotation_to_point((w.their_goal.higher_post - w.their_goal.lower_post) / 2, 0)) < 0.05]
    #lambda w, r: r.has_ball(w.ball),
                     #lambda w, r: utils.can_score(w, r, w.their_goal)]
    preconditions = [lambda w, r: abs(r.get_rotation_to_point(220, 0)) < 0.35]

    def perform(self, comms):
        comms.kick_full_power()
