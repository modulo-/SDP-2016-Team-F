import utils
import math
from logging import info, error

from position import Vector
from models_common import Goal, Action, are_equivalent_positions


# TODO Calibrate these!
ROTATION_THRESHOLD = 0.2
FACING_ROTATION_THRESHOLD = 0.2
ROTATION_TIME_FACTOR = 1
ROTATION_EXTRA_TIME = 0.3
GRAB_DISTANCE = 15
GRAB_DELAY = 2

def is_robot_facing_position(r, pos):
    return abs(utils.attacker_get_rotation_to_point(r.vector, pos)) < ROTATION_THRESHOLD

class GetBall(Goal):
    '''
    Go to and grab ball
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        GoToGrabStaticBall(world, robot),
                        OpenGrabbers(world, robot),
                        GoToBallOpeningDistance(world, robot),
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
    def __init__(self, world, robot):
        self.actions = [TurnToDefenderToReceive(world, robot),
                        GoToScoreZone(world, robot),
                        TurnToScoreZone(world, robot)]
        super(AttackPosition, self).__init__(world, robot)


class AttackerPass(Goal):
    '''
    Pass to defender
    For milestone 3
    '''
    def __init__(self, world, robot):
        self.actions = [KickToDefender(world, robot),
                        TurnToDefenderToGive(world, robot)]
        super(AttackerPass, self).__init__(world, robot)


class AttackerBlock(Goal):
    '''
    Our attacker block attacking opponent
    '''
    def __init__(self, world, robot):
        self.actions = [TurnToBlockingAngle(world, robot),
                        GoToBlockingPosition(world, robot),
                        TurnToFaceBlockingPosition(world, robot)]
        super(AttackerBlock, self).__init__(world, robot)


class GoToGrabStaticBall(Action):
    preconditions = [(lambda w, r: utils.ball_is_static(w), "Ball is static"),
                     (lambda w, r: is_robot_facing_position(r, w.ball.vector), "Attacker is facing ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Attacker's grabbers are open")]

    def perform(self, comms):
        d = None
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        comms.move(d - GRAB_DISTANCE)


class GoToBallOpeningDistance(Action):
    preconditions = [(lambda w, r: is_robot_facing_position(r, w.ball), "Attacker is facing ball")]
    # lambda w, r: r.get_displacement_to_point(w.ball.x, w.ball.y) > 60]

    def perform(self, comms):
        d = None
        if utils.ball_is_static(self.world):
            dx = self.world.ball.x - self.robot.x
            dy = self.world.ball.y - self.robot.y
            d = math.sqrt(dx**2 + dy**2)
        else:
            # TODO Find ball path
            dx = self.world.ball.x - self.robot.x
            dy = self.world.ball.y - self.robot.y
            d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 50
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

    def get_delay(self):
        return GRAB_DELAY

class TurnToGoal(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Attacker has ball")]

    def __init__(self, world, robot):
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, world.their_goal.vector)
        super(TurnToGoal, self).__init__(world, robot)

    def perform(self, comms):
        # TODO find best point to shoot to
        # x = self.world.their_goal.x + self.world.their_goal.width / 2
        # x = (self.world.their_goal.higher_post -
        #     self.world.their_goal.lower_post) / 2
        info("Turning to goal at ({0}, {1})".format(self.world.their_goal.x, self.world.their_goal.y))
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME 

class TurnToDefenderToGive(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Attacker has ball")]

    def __init__(self, world, robot):
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, world.our_defender.vector)
        super(TurnToDefenderToGive, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME

class TurnToBall(Action):
    def __init__(self, world, robot):
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, world.ball.vector)
        super(TurnToBall, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME


class Shoot(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Robot has ball"),
                     (lambda w, r: utils.can_score(w, r, w.their_goal), "Robot can score")]

    def perform(self, comms):
        comms.kick_full_power()

    def get_delay(self):
        print("Getting delay")
        return 10

class KickToDefender(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Robot has ball"),
                     (lambda w, r: is_robot_facing_position(r, w.our_defender.vector), "Robot can score")]

    def perform(self, comms):
        comms.kick_full_power()

    def get_delay(self):
        return 3

class TurnToScoreZone(Action):
    def __init__(self, world, robot):
        position = world.get_new_score_zone()
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, position)
        super(TurnToScoreZone, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME


class GoToScoreZone(Action):
    preconditions = [(lambda w, r: is_robot_facing_position(r, w.get_new_score_zone()),
                      "Attacker is facing score zone")]
    def perform(self, comms):
        raise NotImplementedError


class TurnToDefenderToReceive(Action):
    precondtions = [(lambda w, r: r.are_equivalent_positions(r.vector, w.get_new_score_zone()), "Attacker in score zone")]

    def __init__(self, world, robot):
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, world.their_attackers[0].vector)
        super(TurnToDefenderToReceive, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME


class TurnToFaceBlockingPosition(Action):
    def __init__(self, world, robot):
        position = robot.get_blocking_position(world)
        self.angle = utils.attacker_get_rotation_to_point(robot.vector, position)
        super(TurnToFaceBlockingPosition, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR +ROTATION_EXTRA_TIME 

class GoToBlockingPosition(Action):
    preconditions = [(lambda w, r: is_robot_facing_position(r, r.get_blocking_position(w)),
                      "Attacker is facing blocking position")]

    def perform(self, comms):
        position = self.robot.get_blocking_position(self.world)
        dx = position.x - self.robot.x
        dy = position.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        comms.move(d)


class TurnToBlockingAngle(Action):
    preconditions = [(lambda w, r: are_equivalent_positions(r.vector, r.get_blocking_position(w)),
                     "Attacker in pass blocking position")]

    def __init__(self, world, robot):
        if world.robot_in_possession:
            self.angle = utils.attacker_get_rotation_to_point(robot, world.robot_in_possession.vector)
        else:
            error("Attempting to block while no robot in possession")
            self.angle = 0
        super(TurnToBlockingAngle, self).__init__(world, robot)

    def perform(self, comms):
        comms.turn(self.angle)

    def get_delay(self):
        return abs(self.angle) * ROTATION_TIME_FACTOR + ROTATION_EXTRA_TIME 
