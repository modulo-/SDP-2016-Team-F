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
    def __init__(self, world, robot):
        self.actions = [TurnToBlockingAngle(world, robot),
                        GoToBlockingPoistion(world, robot),
                        TurnToFaceBlockingPosition(world, robot)]

	self.world = world
	self.robot = robot

    def generate_action(self):
        # get our position
	    our_position = (world.our_attacker.x, world.our_attacker.y)
        # get opponents position
	    their_position = (world.their_attackers.x, world.their_attackers.y)
        # get our angle
	    our_angle = world.our_attacker.angle
        # get opponents angle
        their_angle = world.their_attackers.angle
        # generate point to move to
        angle = (math.pi)/2 - our_angle
        point = our_position + (math.cos(angle), math.sin(angle))
        # turn and move robot to point
        (displacement, direction) = world.get_direction_to_point(point)
        comms.turn(direction)
        comms.move(displacement)
        raise NotImplementedError


class GoToBall(Action):
    preconditions = [(lambda w, r: abs(utils.attacker_get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_THRESHOLD, "Attacker is facing ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Attacker's grabbers are open")]

    def perform(self, comms):
        if utils.ball_is_static(self.world):
            dx = self.world.ball.x - self.robot.x
            dy = self.world.ball.y - self.robot.y
            d = math.sqrt(dx**2 + dy**2)
        else:
            # Find ball path
            pass

        # TODO grabbing area size
        grabber_size = 30
        comms.move(d - grabber_size)


class GoToOpeningDistanceStaticBall(Action):
    preconditions = [(lambda w, r: utils.ball_is_static(w), "Ball is static"),
                     (lambda w, r: abs(utils.attacker_get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_THRESHOLD, "Attacker is facing ball")]
    # lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) > 60]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 70
        comms.move(d - grabber_size)


class OpenGrabbers(Action):
    preconditions = [(lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) < 80, "Ball is in attacker's grabber range"),
                     (lambda w, r: r.catcher == 'CLOSED', "Attacker's grabbers are closed")]

    def perform(self, comms):
        comms.release_grabbers()


class GrabBall(Action):
    preconditions = [(lambda w, r: r.can_catch_ball(w.ball), "Attacker can catch ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Catchers open")]

    def perform(self, comms):
        comms.close_grabbers()


class TurnToGoal(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Attacker has ball")]

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
        comms.turn(utils.attacker_get_rotation_to_point(self.robot.vector, self.world.ball.vector))


class Shoot(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Robot has ball"),
                     (lambda w, r: utils.can_score(w, r, w.their_goal), "Robot can score")]

    def perform(self, comms):
        comms.kick_full_power()


class GoToScoreZone(Action):
    def perform(self, comms):
        raise NotImplementedError


class TurnToDefender(Action):
    precondtions = [(lambda w, r: r.are_equivalent_positions(r.vector, r.score_zone), "Attacker in score zone")]

    def perform(self, comms):
        raise NotImplementedError


class TurnToFaceBlockingPosition(Action):
    def perform(self, comms):
        position = self.robot.get_blocking_position(self.world)
        rotation = utils.attacker_get_rotation_to_point(self.robot.vector, position)
        comms.turn(rotation)


class GoToBlockingPosition(Action):
    preconditions = [(lambda w, r: abs(utils.attacker_get_rotation_to_point(r.vector, r.get_blocking_position(w)))
                      < ROTATION_THRESHOLD, "Attacker is facing blocking position")]

    def perform(self, comms):
        position = self.robot.get_blocking_position(self.world)
        dx = position.x - self.robot.x
        dy = position.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        comms.move(d)


class TurnToBlockingAngle(Action):
    precondtions = [(lambda w, r: r.is_pass_blocking_position(w),
                     "Attacker in pass blocking position")]

    def perform(self, comms):
        raise NotImplementedError
