import utils
import math
import logging

from position import Vector
from models_common import Goal, Action
from time import time


'''
> CONSTANTS
'''


GOAL_RADIUS = 100
DEFEND_POINT_DISTANCE_TTHRESHOLD = 20
ROTATION_BALL_THRESHOLD = 0.2
ROTATION_THRESHOLD = 0.2
WIGGLE_EFFECT = 15
FOLLOW_BALL_DISTANCE_THRESHOLD = 50
ROTATION_DEFEND_BALL_THRESHOLD = 0.4
MOVEMENT_THRESHOLD = 15
FACING_ROTATION_THRESHOLD = 0.2
CLOSE_DISTANCE_BALL_THRESHOLD = 50
MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD = 75


'''
> GOALS
'''


class Defend(Goal):
    '''
    The first task of Milestone 3
    Receiving a pass
    '''

    def __init__(self, world, robot):
        self.start_time = time()

        self.actions = [GrabBall(world, robot),
                        GoToDefendPoint(world, robot),
                        RotateToDefendPoint(world, robot),
                        FaceBall(world, robot),
                        Wiggle(world, robot),
                        OpenGrabbers(world, robot)
                        ]
        super(Defend, self).__init__(world, robot)


class Pass(Goal):
    '''
    Pass to attacker
    '''
    def __init__(self, world, robot):
        self.actions = [PassAction(world, robot)]
        super(Pass, self).__init__(world, robot)


class GetBall(Goal):
    '''
    Go to and grab ball
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        GoToStaticBall(world, robot),
                        TurnToCatchPoint(world, robot),
                        OpenGrabbers(world, robot)]
        super(GetBall, self).__init__(world, robot)

'''
> ACTIONS
'''


class GrabBall(Action):
    '''
    Grab ball
    '''
    preconditions = [
        (lambda w, r: r.can_catch_ball(w.ball), "Defender can catch ball"),
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        comms.close_grabbers()
        self.robot.catcher = 'CLOSED'


class GoToDefendPoint(Action):
    preconditions = [
        (lambda w, r: abs(utils.defender_rotation_to_defend_point(r.vector, w.ball.vector, w.our_goal.vector, GOAL_RADIUS)[0]) < ROTATION_DEFEND_POINT_THRESHOLD, "Defender is facing defending point"),
        (lambda w, r: abs(utils.defender_distance_to_defend_point(r.vector, w.ball.vector, w.our_goal.vector, GOAL_RADIUS)) > DEFEND_POINT_DISTANCE_THRESHOLD, "Defender far away from defending point"),
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        angle, side, defend_point = utils.defender_rotation_to_defend_point(self.robot.vector, self.world.ball.vector, self.world.our_goal.vector, GOAL_RADIUS)

        dx = defend_point.x - self.robot.x
        dy = defend_point.y - self.robot.y
        distance_to_move = math.hypot(dx, dy)

        if side == "left":
            distance_to_move = -distance_to_move

        logging.info("Wants to move by: " + str(distance_to_move))
        comms.move(distance_to_move)


class RotateToDefendPoint(Action):
    preconditions = [
        (lambda w, r: abs(utils.defender_distance_to_defend_point(r.vector, w.ball.vector, w.our_goal.vector, GOAL_RADIUS)) > DEFEND_POINT_DISTANCE_THRESHOLD, "Defender far away from defending point"),
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        angle, side, defend_point = utils.defender_rotation_to_defend_point(self.robot.vector, self.world.ball.vector, self.world.our_goal.vector, GOAL_RADIUS)

        logging.info("Facing defend point: Rotating %f degrees to %f %f" % (math.degrees(angle), defend_point.x, defend_point.y))
        comms.turn(angle)


class FaceBall(Action):
    preconditions = [
        (lambda w, r: abs(utils.get_rotation_to_point(r.vector, w.ball.vector)) > ROTATION_DEFEND_BALL_THRESHOLD, "Defender is not facing ball"),
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        angle = utils.get_rotation_to_point(self.robot.vector, self.world.ball.vector)
        logging.info("Facing ball: Rotating %f" % angle)
        comms.turn(angle)


class Wiggle(Action):
    preconditions = [
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        angle, side, defend_point = utils.defender_rotation_to_defend_point(self.robot.vector, self.world.ball.vector, self.world.our_goal.vector, GOAL_RADIUS)

        dx = defend_point.x - self.robot.x
        dy = defend_point.y - self.robot.y
        wiggeled_distance_to_move = math.hypot(dx, dy) + WIGGLE_EFFECT

        if side == "left":
            wiggeled_distance_to_move = -wiggeled_distance_to_move

        logging.info("Wants to wiggle by: " + str(wiggeled_distance_to_move))
        comms.move(wiggeled_distance_to_move)


class FollowBall(Action):
    preconditions = [
        (lambda w, r: abs(utils.defender_distance_to_ball(r.vector, w.ball.vector) < FOLLOW_BALL_DISTANCE_THRESHOLD, "Defender close to kicked ball")
    ]

    def perform(self, comms):
        distance_to_move = defender_fellow_ball_distance(self.robot.vector, self.world.ball.vector)

        logging.info("Wants to follow ball by: " + str(distance_to_move))
        comms.move(distance_to_move)


class GoToStaticBall(Action):
    '''
    Move defender to the ball when static
    '''
    preconditions = [
        (lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.ball.x, w.ball.y, 0, 0), r.catch_distance)[0]) < ROTATION_THRESHOLD, "Defender is facing ball"),
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
        (lambda w, r: utils.ball_is_static(w), "Ball is static.")
    ]

    def perform(self, comms):
        # reset dynamic threshold
        # TODO: has to be restarted everywhere!
        global ROTATION_THRESHOLD
        ROTATION_THRESHOLD = 0.1

        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        displacement = math.hypot(dx, dy)

        if displacement == 0:
            alpha = 0
        else:
            # TODO: has to handle this edge case better
            try:
                alpha = math.degrees(math.asin(self.robot.catch_distance / float(displacement)))
            except ValueError as er:
                print(er)
                x = self.world.ball.x
                y = self.world.ball.y
                angle = utils.get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0))
                logging.info("Wants to close-rotate by: " + str(angle))
                comms.turn(angle)
                return 1  # TODO: NO FIXED DELAY
        beta = 90 - alpha

        if math.sin(math.radians(alpha)) == 0:
            distance_to_move = 15  # something is obviously wrong so we have to move a bit
        else:
            distance_to_move = math.sin(math.radians(beta)) * self.robot.catch_distance / math.sin(math.radians(alpha))

        # Find the movement side
        angle, side = utils.defender_get_rotation_to_catch_point(Vector(self.robot.x, self.robot.y, self.robot.angle, 0), Vector(self.world.ball.x, self.world.ball.y, 0, 0), self.robot.catch_distance)
        if side == "left":
            distance_to_move = -distance_to_move

        logging.info("Wants to move by: " + str(distance_to_move))
        comms.move(distance_to_move)

        return utils.defender_move_delay(distance_to_move)


class TurnToBall(Action):
    preconditions = [
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0))
        logging.info("Wants to rotate by: " + str(angle))
        comms.turn(angle)

        return utils.defender_turn_delay(angle)


class TurnToCatchPoint(Action):
    '''
    Turn to point for catching ball
    '''
    preconditions = [
        (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")
    ]

    def perform(self, comms):
        logging.info('---- ' + str(ROTATION_THRESHOLD))
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.defender_get_rotation_to_catch_point(Vector(self.robot.x, self.robot.y, self.robot.angle, 0), Vector(x, y, 0, 0), self.robot.catch_distance)[0]
        logging.info("Wants to rotate by: " + str(angle))

        comms.turn(angle)

        global ROTATION_THRESHOLD
        if ROTATION_THRESHOLD < 0.30:
            ROTATION_THRESHOLD += 0.07

        return utils.defender_turn_delay(angle)


class OpenGrabbers(Action):
    preconditions = [
        (lambda w, r: r.catcher == 'CLOSED', "Grabbers are closed.")
    ]

    def perform(self, comms):
        comms.release_grabbers()
        self.robot.catcher = 'OPEN'

        return 0.5


class PassAction(Action):
    def perform(self, comms):
        target_rotation = utils.defender_scan_angle_to_pass_absolute(self.world, self.robot)
        logging.info("Passing ball. (Rotate %f degrees, then kick)", math.degrees(target_rotation))
        comms.turn_then_kick(target_rotation)
        self.robot.catcher = 'OPEN'
        return 1 + utils.defender_turn_delay(target_rotation)
