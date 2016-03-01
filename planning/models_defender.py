import utils
import math
import logging

from position import Vector
from models_common import Goal, Action, are_equivalent_positions


'''
> CONSTANTS
'''


ROTATION_BALL_THRESHOLD = 0.2
ROTATION_THRESHOLD = 0.2
FACING_ROTATION_THRESHOLD = 0.2
CLOSE_DISTANCE_BALL_THRESHOLD = 50

'''
> GOALS
'''


class ReceivingPass(Goal):
    '''
    The first task of Milestone 3
    Receiving a pass
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        # GoToStaticBall(world, robot)
                        WaitForBallToCome(world, robot),
                        TurnToBall(world, robot),
                        FollowBall(world, robot)]
        super(ReceivingPass, self).__init__(world, robot)


class GetBall(Goal):
    '''
    Go to and grab ball
    '''

    def __init__(self, world, robot):
        self.actions = [GrabBall(world, robot),
                        # TurnToBallIfClose(world, robot),
                        GoToStaticBall(world, robot),
                        TurnToCatchPoint(world, robot)]
        super(GetBall, self).__init__(world, robot)


class DefendGoal(Goal):
    '''
    Move around goal to block attacker
    '''
    def __init__(self, world, robot):
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
        self.actions = [KickToScoreZone(world, robot),
                        TurnToScoreZone(world, robot)]
        super(Goal, self).__init__(world, robot)


class Tactical(Goal):
    '''
    Move to optimum defensive position
    '''
    def __init__(self, world, robot):
        self.actions = [TurnToTacticalDefenceAngle(world, robot),
                        MoveToTacticalDefencePosition(world, robot)]
        super(Tactical, self).__init__(world, robot)


'''
> ACTIONS
'''


class WaitForBallToCome(Action):
    '''
    Defender just waits for the ball to approach it
    '''

    preconditions = [(lambda w, r: abs(utils.get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_BALL_THRESHOLD, "Defender is facing ball"),
                     (lambda w, r: utils.ball_can_reach_robot(w.ball), "The ball can reach the robot"),
                     (lambda w, r: utils.robot_can_reach_ball(w.ball), "Defender can reach the ball")]

    def perform(self, comms):
        pass


class FollowBall(Action):
    '''
    Be in the trajectory of moving ball as long as it's predicted
    that the ball reaches the robot
    '''

    preconditions = [(lambda w, r: abs(utils.get_rotation_to_point(r.vector, w.ball.vector)) < ROTATION_BALL_THRESHOLD, "Defender is facing ball")]

    def perform(self, comms):
        pass


class TurnToBallIfClose(Action):
    preconditions = [(lambda w, r: math.hypot(self.robot.vector, self.world.ball.vector) < CLOSE_DISTANCE_BALL_THRESHOLD, "Defender is close to a ball")]

    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0))
        logging.info("Wants to close-rotate by: " + str(angle))
        comms.turn(angle)


class TurnToBall(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.get_rotation_to_point(self.robot.vector, Vector(x, y, 0, 0))
        logging.info("Wants to rotate by: " + str(angle))
        comms.turn(angle)


class GoToStaticBall(Action):
    '''
    Move defender to the ball when static
    '''
    preconditions = [(lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.ball.x, w.ball.y, 0, 0), r.catch_distance)[0]) < ROTATION_THRESHOLD, "Defender is facing ball")]

    def perform(self, comms):
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
                return 0
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


class GrabBall(Action):
    '''
    Grab ball
    '''
    preconditions = [(lambda w, r: r.can_catch_ball(w.ball), "Defender can catch ball")]

    def perform(self, comms):
        comms.close_grabbers()


class TurnToCatchPoint(Action):
    '''
    Turn to point for catching ball
    '''
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        angle = utils.defender_get_rotation_to_catch_point(Vector(self.robot.x, self.robot.y, self.robot.angle, 0), Vector(x, y, 0, 0), self.robot.catch_distance)[0]
        logging.info("Wants to rotate by: " + str(angle))
        comms.turn(angle)


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
