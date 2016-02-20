import utils
import math

from position import Vector
from models_common import Goal, Action, ROTATION_THRESHOLD,\
    FACING_ROTATION_THRESHOLD, are_equivalent_positions

class GetBall(Goal):
    '''
    Go to and grab ball
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        GoToStaticBall(world, robot),
                        OpenGrabbers(world, robot),
                        GoToOpeningDistanceStaticBall(world, robot),
                        TurnToBall(world, robot)]
        super(GetBall, self).__init__(world, robot)


class Score(Goal):
    '''
    Turn and shoot
    '''

    def __init__(self, world, robot):
        self.actions = [Shoot(world, robot),
                        TurnToGoal(world, robot)]
        super(Score, self).__init__(world, robot)


class AttackPosition(Goal):
    '''
    Move to optimum attacking position
    '''
    def generate_action(self):
        raise NotImplementedError


class AttackBlock(Goal):
    '''
    Our attacker block attacking opponent
    '''
    def generate_action(self):
        raise NotImplementedError


class GoToStaticBall(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(utils.attacker_get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_THRESHOLD,
                     lambda w, r: r.catcher == 'OPEN']

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)

        # TODO grabbing area size
        grabber_size = 30
        comms.move(d - grabber_size)


class GoToOpeningDistanceStaticBall(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(utils.attacker_get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_THRESHOLD]
    # lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) > 60]

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
    preconditions = [lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        # TODO find best point to shoot to
        # x = self.world.their_goal.x + self.world.their_goal.width / 2
        # x = (self.world.their_goal.higher_post -
        #     self.world.their_goal.lower_post) / 2
        x = 220
        y = 0  # self.world.their_goal.y
        comms.turn(self.robot.attacker_get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0)))


class TurnToBall(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(utils.attacker_get_rotation_to_point(Vector(self.robot.vetor, Vector(x, y, 0, 0))))


class Shoot(Action):
    preconditions = [lambda w, r: r.has_ball(w.ball),
                     lambda w, r: utils.can_score(w, r, w.their_goal)]

    def perform(self, comms):
        comms.kick_full_power()
