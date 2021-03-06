import utils
import math
import logging

from position import Vector
from models_common import Goal, Action, are_equivalent_positions
from math import pi
from time import time



'''
> CONSTANTS
'''


ROTATION_DEFEND_POINT_THRESHOLD = 0.2
ROTATION_BALL_THRESHOLD = 0.2
ROTATION_THRESHOLD = 0.10
MOVEMENT_THRESHOLD = 10
DEFENCE_AREA_THRESHOLD = 30
FACING_ROTATION_THRESHOLD = 0.2
CLOSE_DISTANCE_BALL_THRESHOLD = 50
MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD = 45

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
                        FaceBall(world, robot)
                        ]
        super(Defend, self).__init__(world, robot)


class ReceivingPass(Goal):
    '''
    The first task of Milestone 3
    Receiving a pass
    '''

    def __init__(self, world, robot):
        self.start_time = time()

        self.actions = [GrabBall(world, robot),
                        GoToStaticBall(world, robot, [
                            (lambda w, r: math.hypot(w.our_attacker.x - w.ball.x, w.our_attacker.y - w.ball.y) > MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD, "The house robot has the ball")]),
                        TurnToCatchPoint(world, robot, [
                            (lambda w, r: math.hypot(w.our_attacker.x - w.ball.x, w.our_attacker.y - w.ball.y) > MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD, "The house robot has the ball")]),
                        WaitForBallToCome(world, robot),
                        FollowBall(world, robot),
                        OpenGrabbers(world, robot),
                        FaceFriendly(world, robot)]
        super(ReceivingPass, self).__init__(world, robot)


def opponent_doesnt_have_ball(w, robot):
    return not any(math.hypot(r.x - w.ball.x, r.y - w.ball.y) < MILESTONE_BALL_AWAY_FROM_HOUSEROBOT_THRESHOLD for r in w.their_robots)


class ReceiveAndPass(Goal):
    '''
    The second task of Milestone 3
    '''

    def __init__(self, world, robot):
        if robot.catched_ball:
            print("222222222")
            self.actions = [Kick(world, robot, [
                                (lambda w, r: abs(utils.get_rotation_to_point(r.vector, w.our_attacker)) < ROTATION_BALL_THRESHOLD, "Defender is facing attacker.")]),
                            FaceFriendly(world, robot, [])]

        else:
            print("111111111")
            self.actions = [FaceFriendly(world, robot, [
                            (lambda w, r: r.has_ball(w.ball), "Our robot has the ball")]),
                            GrabBall(world, robot),
                            GoToStaticBall(world, robot, [
                                (opponent_doesnt_have_ball, "The house robot has the ball")]),
                            TurnToCatchPoint(world, robot, [
                                (opponent_doesnt_have_ball, "The house robot has the ball")]),
                            OpenGrabbers(world, robot),
                            FaceOpponent(world, robot)]


class InterceptPass(Goal):
    '''
    Task 3.1 of Milestone 3
    Intercept a pass between two opponents
    '''

    def __init__(self, world, robot):
        self.actions = [AlignForPassIntercept(world, robot)]
        super(InterceptPass, self).__init__(world, robot)


class InterceptGoal(Goal):
    '''
    Task 3.2 of Milestone 3
    Intercept a goal shoot of an opponent.
    '''

    def __init__(self, world, robot):
        self.actions = [AlignForGoalIntercept(world, robot)]
        super(InterceptGoal, self).__init__(world, robot)


class GetBall(Goal):
    '''
    Go to and grab ball
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        # TurnToBallIfClose(world, robot),
                        GoToStaticBall(world, robot),
                        TurnToCatchPoint(world, robot),
                        OpenGrabbers(world, robot)]
        super(GetBall, self).__init__(world, robot)


class DefendGoal(Goal):
    '''
    Move around goal to block attacker
    '''
    def __init__(self, world, robot):
        # FIXME: What does the OpenGrabbersForOpponentShot try to do?
        # The grabbers should be open anyway.
        self.actions = [OpenGrabbersForOpponentShot(world, robot),
                        TurnToOpposingAttacker(world, robot),
                        MoveOnGoalArc(world, robot),
                        MoveToGoalArc(world, robot)]
        super(DefendGoal, self).__init__(world, robot)


class Pass(Goal):
    '''
    Pass to attacker
    '''
    def __init__(self, world, robot):
        self.actions = [PassAction(world, robot)]
        super(Pass, self).__init__(world, robot)


class Tactical(Goal):
    '''
    Move to optimum defensive position
    '''
    def __init__(self, world, robot):
        self.actions = [TurnToTacticalDefenceAngle(world, robot),
                        MoveToTacticalDefencePosition(world, robot)]
        super(Tactical, self).__init__(world, robot)


class Block(Goal):
    def __init__(self, world, robot):
        self.actions = [RotateAndAlignForBlock(world, robot)]
        super(Block, self).__init__(world, robot)


class ReactiveGrabGoal(Goal):
    def __init__(self, world, robot):
        self.actions = [AlignForGrab(world, robot),
                        RotateAndAlignForGrab(world, robot),
                        ReactiveGrabAction(world, robot)]
        super(ReactiveGrabGoal, self).__init__(world, robot)


class ReturnToDefenceArea(Goal):
    def __init__(self, world, robot):
        self.actions = [FaceOppositePitchSide(world, robot),
                        MoveToDefenceArea(world, robot),
                        RotateToDefenceArea(world, robot)]





'''
> ACTIONS
'''


class GoToDefendPoint(Action):
    preconditions = [(lambda w, r: abs(utils.defender_rotation_to_defend_point(r.vector, w.ball.vector, w.our_goal.vector, 100)[0]) < ROTATION_DEFEND_POINT_THRESHOLD, "Defender is facing defending point"),
                        ]

    def perform(self, comms):
        angle, side, defend_point = utils.defender_rotation_to_defend_point(self.robot.vector, self.world.ball.vector, self.world.our_goal.vector, 100)

        dx = defend_point.x - self.robot.x
        dy = defend_point.y - self.robot.y
        distance_to_move = math.hypot(dx, dy)

        if side == "left":
            distance_to_move = -distance_to_move

        logging.info("Wants to move by: " + str(distance_to_move))
        comms.move(distance_to_move)

class RotateToDefendPoint(Action):
    preconditions = [(lambda w, r: abs(utils.defender_distance_to_defend_point(r.vector, w.ball.vector, w.our_goal.vector, 100)) > 20, "Defender close to defending point")]

    def perform(self, comms):
        angle, side, defend_point = utils.defender_rotation_to_defend_point(self.robot.vector, self.world.ball.vector, self.world.our_goal.vector, 100)

        logging.info("Facing defend point: Rotating %f degrees to %f %f" % (math.degrees(angle), defend_point.x, defend_point.y))
        comms.turn(angle)


class FaceBall(Action):
    preconditions = []

    def perform(self, comms):
        angle = utils.get_rotation_to_point(self.robot.vector, self.world.ball.vector)
        logging.info("Facing ball: Rotating %f" % angle)
        comms.turn(angle)


class GoToStaticBall(Action):
    '''
    Move defender to the ball when static
    '''
    preconditions = [(lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.ball.x, w.ball.y, 0, 0), r.catch_distance)[0]) < ROTATION_THRESHOLD, "Defender is facing ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
                     (lambda w, r: utils.ball_is_static(w), "Ball is static.")]

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
    preconditions = [(lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")]

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
    preconditions = [(lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")]

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


class PassAction(Action):
    def perform(self, comms):
        target_rotation = utils.defender_scan_angle_to_pass_absolute(self.world, self.robot)
        logging.info("Passing ball. (Rotate %f degrees, then kick)", math.degrees(target_rotation))
        comms.turn_then_kick(target_rotation)
        self.robot.catcher = 'OPEN'
        return 1 + utils.defender_turn_delay(target_rotation)


class FaceOppositePitchSide(Action):
    preconditions = [(lambda w, r: abs(utils.dist(r.vector, utils.get_defence_point(w))) < DEFENCE_AREA_THRESHOLD, "At defence point.")]

    def perform(self, comms):
        if self.world.our_side == 'left':
            target_angle = pi / 2
        else:
            target_angle = 3 * pi / 2
        print target_angle
        rot_angle = (target_angle - self.robot.angle + pi) % (2 * pi) - pi
        logging.info("Facing opposite pitch side. Rotating %f degrees.", math.degrees(rot_angle))
        comms.turn(rot_angle)
        return utils.defender_turn_delay(rot_angle)


class MoveToDefenceArea(Action):

    preconditions = [(lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), utils.get_defence_point(w), 0)[0]) < ROTATION_THRESHOLD, "Aligned to move to defence point.")]

    def perform(self, comms):
        dist = utils.dist(self.robot, utils.get_defence_point(self.world))
        dist *= utils.get_movement_direction_from_vector(self.robot, utils.get_defence_point(self.world))
        logging.info("Moving to defence area. Moving %f right.", dist)
        comms.move(dist)
        return utils.defender_move_delay(abs(dist))


class RotateToDefenceArea(Action):

    def perform(self, comms):
        rot_angle = utils.defender_get_rotation_to_catch_point(
            Vector(self.robot.x, self.robot.y, self.robot.angle, 0),
            utils.get_defence_point(self.world), 0)[0]
        logging.info("Facing direction to move to defence area. Rotating %f degrees.", math.degrees(rot_angle))
        comms.turn(rot_angle)
        return utils.defender_turn_delay(rot_angle)


class RotateAndAlignForBlock(Action):
    def perform(self, comms):
        goal = self.world.our_goal
        robots = filter(lambda r: not r.is_missing(), self.world.their_robots)
        robots.sort(key=lambda r: math.hypot(r.x - goal.x, r.y - goal.y))
        if len(robots) == 0:
            logging.error("There is no enemy here. Gimme someone to destroy!")
            return 1
        robot = robots[0]
        cone_upper = math.atan2(goal.x - robot.x, goal.higher_post - robot.y) % (2 * pi)
        cone_lower = math.atan2(goal.x - robot.x, goal.lower_post - robot.y) % (2 * pi)
        if robot.angle < min(cone_upper, cone_lower) or robot.angle > max(cone_upper, cone_lower):
            critical_angle = (cone_upper + cone_lower) / 2
        else:
            critical_angle = robot.angle

        target_rotation = (critical_angle - self.robot.angle + pi / 2) % pi - pi / 2
        if (critical_angle - self.robot.angle + pi / 2) % pi - pi / 2 <= 0.6:
            target_rotation = 0
        dist = utils.defender_get_alignment_offset(self.robot, robot, critical_angle, target_rotation)
        logging.info("Aligning with enemy. (Rotate %f degrees, move %f right)", math.degrees(target_rotation), dist)
        if abs(dist) > MOVEMENT_THRESHOLD and abs(target_rotation) > ROTATION_THRESHOLD:
            comms.turn_then_move(target_rotation, dist)
            return utils.defender_turn_delay(target_rotation) + utils.defender_move_delay(dist)
        elif abs(dist) > MOVEMENT_THRESHOLD:
            comms.move(dist)
            return utils.defender_move_delay(dist)
        elif abs(target_rotation) > ROTATION_THRESHOLD:
            comms.turn(target_rotation)
            return utils.defender_turn_delay(target_rotation)


class RotateAndAlignForGrab(Action):
    preconditions = [(lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
                     (lambda w, r: abs((r.angle - w.ball.angle + pi / 2) % pi - pi / 2) >= 0.6, "Ball vector and robot vector not within 35 degrees."),
                     (lambda w, r: not utils.ball_is_static(w), "The ball is moving")]

    def perform(self, comms):
        target_rotation = (self.world.ball.angle - self.robot.angle + pi / 2) % pi - pi / 2
        dist = utils.defender_get_alignment_offset(self.robot, self.world.ball, self.world.ball.angle, target_rotation)
        logging.info("Aligning with ball. (Rotate %f degrees, move %f right)", math.degrees(target_rotation), dist)
        if abs(dist) > MOVEMENT_THRESHOLD:
            comms.turn_then_move(target_rotation, dist)
            return utils.defender_turn_delay(target_rotation) + utils.defender_move_delay(dist)
        else:
            comms.turn(target_rotation)
            return utils.defender_turn_delay(target_rotation)


class AlignForGrab(Action):
    preconditions = [(lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
                     (lambda w, r: abs((r.angle - w.ball.angle + pi / 2) % pi - pi / 2) < 0.6, "Ball vector and robot vector within 35 degrees."),
                     (lambda w, r: not utils.ball_is_static(w), "The ball is moving")]

    def perform(self, comms):
        dist = utils.defender_get_alignment_offset(self.robot, self.world.ball, self.world.ball.angle)
        logging.info("Aligning with ball. (Move %f right)", dist)
        if abs(dist) > MOVEMENT_THRESHOLD:
            comms.move(dist)
            return utils.defender_move_delay(dist)


class ReactiveGrabAction(Action):
    preconditions = [(lambda w, r: r.can_catch_ball(w.ball), "Robot can catch the ball."),
                     (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")]

    def perform(self, comms):
        logging.info("Grabbing.")
        self.robot.catcher = 'CLOSED'
        comms.close_grabbers()
        return 1


class Kick(Action):
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Robot has the ball.")]

    def perform(self, comms):
        logging.info("Kicking.")
        comms.kick_full_power()
        self.robot.catcher = 'OPEN'
        return 1


class AlignForPassIntercept(Action):
    preconditions = [(lambda w, r: all(not r.is_missing() for r in w.their_robots), "Both enemies are on the pitch.")]

    def perform(self, comms):
        robots = self.world.their_robots
        y_diff = robots[1].y - robots[0].y
        x_diff = robots[1].x - robots[0].x
        ratio = (self.world.our_defender.x - robots[0].x) / x_diff

        y_mean = robots[0].y + (y_diff * ratio)

        distance = utils.defender_distance_on_y(self.world.our_defender.vector, y_mean)

        print("DISTANCE: " + str(distance))
        logging.info("Wants to move by: " + str(distance))
        comms.move(distance)
        return utils.defender_move_delay(distance)


class AlignForGoalIntercept(Action):
    preconditions = [(lambda w, r: all(not r.is_missing() for r in w.their_robots), "Both enemies are on the pitch.")]

    def perform(self, comms):
        # get robot
        robots = filter(lambda r: not r.is_missing(), self.world.their_robots)
        robots.sort(key=lambda r: math.hypot(r.x - self.robot.x, r.y - self.robot.y))
        if len(robots) == 0:
            logging.error("There is no enemy here. Gimme someone to destroy!")
            return 1
        robot = robots[0]

        # Find our goal
        goal = Vector(0, 225, 0, 0)
        if self.robot.x > 300:
            goal = Vector(600, 225, 0, 0)

        # get the point
        robots = self.world.their_robots
        y_diff = goal.y - robot.y
        x_diff = goal.x - robot.x
        ratio = (self.world.our_defender.x - robot.x) / x_diff

        y_mean = robot.y + (y_diff * ratio)

        distance = utils.defender_distance_on_y(self.world.our_defender.vector, y_mean)

        print("DISTANCE: " + str(distance))
        logging.info("Wants to move by: " + str(distance))
        comms.move(distance)

        return utils.defender_move_delay(distance)


class WaitForBallToCome(Action):
    '''
    Defender just waits for the ball to approach it
    '''

    # FIXME: ball_can_reach_robot doesn't do anything.
    # FIXME: robot_can_reach_ball doesn't do anything.
    preconditions = [(lambda w, r: abs(utils.get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_BALL_THRESHOLD, "Defender is facing ball"),
                     (lambda w, r: utils.ball_can_reach_robot(w.ball, r), "The ball can reach the robot"),
                     (lambda w, r: utils.robot_can_reach_ball(w.ball, r), "Defender can reach the ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
                     (lambda w, r: not utils.ball_is_static(w), "The ball is moving.")]

    def perform(self, comms):
        pass


class FollowBall(Action):
    '''
    Be in the trajectory of moving ball as long as it's predicted
    that the ball reaches the robot
    '''

    # Different precondition: Face counter to ball trajectory instead of facing
    # the ball (allows intercepting as well as possible.
    preconditions = [(lambda w, r: r.catcher == 'OPEN', "Grabbers are open."),
                     (lambda w, r: not utils.ball_is_static(w), "The ball is moving.")]

    def perform(self, comms):
        turn_angle = (self.world.ball.angle - self.robot.angle) % (2 * pi) - pi
        dist = math.hypot(self.robot.x - self.world.ball.x, self.robot.y - self.world.ball.y)
        tri_angle = utils.get_rotation_to_point(self.world.ball.vector, self.robot)
        dist *= math.cos(tri_angle)
        if abs(turn_angle) < ROTATION_BALL_THRESHOLD:
            logging.info("Adjusting to ball position by distance: " + str(dist))
            comms.move(dist)
        else:
            logging.info("Adjusting angle by %f, distance by %f." % (turn_angle, dist))
            comms.turn_then_move(turn_angle, dist)
        return 2


class FaceOpponent(Action):
    preconditions = [(lambda w, r: any(not r.is_missing() for r in w.their_robots), "Enemy robot is on the pitch.")]

    def perform(self, comms):
        # takes take opponent's robot that is closer to ours
        robots = filter(lambda r: not r.is_missing(), self.world.their_robots)
        robots.sort(key=lambda r: math.hypot(r.x - self.robot.x, r.y - self.robot.y))
        if len(robots) == 0:
            logging.error("There is no enemy here. Gimme someone to destroy!")
            return 1
        robot = robots[0]
        angle = utils.get_rotation_to_point(self.robot.vector, robot.vector)
        logging.info("Facing opponent: Rotating %f" % angle)
        comms.turn(angle)


class FaceFriendly(Action):
    preconditions = [(lambda w, r: not w.our_attacker.is_missing(), "Friendly is on the pitch.")]

    def perform(self, comms):
        self.robot._catched_ball = True
        angle = utils.get_rotation_to_point(self.robot.vector, self.world.our_attacker.vector)
        logging.info("Facing friendly: Rotating %f" % angle)
        comms.turn(angle)


class TurnToBallIfClose(Action):
    preconditions = [(lambda w, r: math.hypot(r.vector, w.ball.vector) < CLOSE_DISTANCE_BALL_THRESHOLD, "Defender is close to a ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")]

    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0))
        logging.info("Wants to close-rotate by: " + str(angle))
        comms.turn(angle)


class OpenGrabbers(Action):
    preconditions = [(lambda w, r: r.catcher == 'CLOSED', "Grabbers are closed.")]

    def perform(self, comms):
        comms.release_grabbers()
        self.robot.catcher = 'OPEN'

        return 0.5


class GrabBall(Action):
    '''
    Grab ball
    '''
    preconditions = [(lambda w, r: r.can_catch_ball(w.ball), "Defender can catch ball"),
                     (lambda w, r: r.catcher == 'OPEN', "Grabbers are open.")]

    def perform(self, comms):
        comms.close_grabbers()
        self.robot.catcher = 'CLOSED'


class Shoot(Action):
    '''
    Defender shoots?
    '''
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Defender has ball"),
                     (lambda w, r: utils.can_score(w, r, w.their_goal), "Defender can score")]

    def perform(self, comms):
        comms.kick_full_power()


class MoveToGoalArc(Action):
    '''
    Move to goal defending position, an arc around our goal
    '''
    def perform(self, comms):
        raise NotImplementedError


class MoveOnGoalArc(Action):
    '''
    Move around our goal arc to defend goal
    '''
    preconditions = [(lambda w, r: r.on_goal_arc(w.our_goal), "Defender is on goal arc")]

    def perform(self, comms):
        raise NotImplementedError


class TurnToOpposingAttacker(Action):
    '''
    Turn to face opponent's attacker
    '''
    preconditions = [(lambda w, r: r.aligned_on_goal_arc(w.robot_in_possession), "Defender is aligned with robot in possession on goal arc")]

    def perform(self, comms):
        raise NotImplementedError


class OpenGrabbersForOpponentShot(Action):
    '''
    Open grabbers to block oponnent's shot
    '''
    def rotation_precondition(self, w, r):
        possessing = w.robot_in_posession
        rotation = utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(possessing.x, possessing.y, 0, 0), r.catch_distance)[0]
        return abs(rotation) < FACING_ROTATION_THRESHOLD

    preconditions = [(lambda w, r: r.aligned_on_goal_arc(w.robot_in_possession), "Defender is aligned with robot in possession on goal arc"),
                     (rotation_precondition, "Defender is facing robot in possession on goal arc")]

    def perform(self, comms):
        raise NotImplementedError


class TurnToScoreZone(Action):
    '''
    Turn with ball to prepare pass to attacker's score zone
    '''
    preconditions = [(lambda w, r: r.has_ball(w.ball), "Defender is facing attacker's score zone")]

    def perform(self, comms):
        raise NotImplementedError


class KickToScoreZone(Action):
    '''
    Pass ball to our attacker's score zone
    '''
    preconditions = [(lambda w, r: abs(r=utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0)[0], Vector(w.score_zone.x, w.score_zone.y, 0, 0), r.catch_distance)) < FACING_ROTATION_THRESHOLD,
                      "Defender is facing attacker's score zone"),
                     (lambda w, r: r.has_ball(w.ball), "Defender has ball")]

    def perform(self, comms):
        raise NotImplementedError


class MoveToTacticalDefencePosition(Action):
    '''
    Move defender to tactical position
    '''

    def perform(self, comms):
        raise NotImplementedError


class TurnToTacticalDefenceAngle(Action):
    '''
    Turn defender to tactical angle
    '''
    preconditions = [(lambda w, r: are_equivalent_positions(r, r.tactical_position), "Defender is in tactical position")]

    def perform(self, comms):
        raise NotImplementedError
