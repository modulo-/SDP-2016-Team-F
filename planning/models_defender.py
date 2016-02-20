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


class GoToStaticBall(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.ball.x, w.ball.y, 0, 0), r.catch_distance)) < ROTATION_THRESHOLD]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        displacement = math.hypot(dx, dy)
        print(displacement)

        if displacement == 0:
            alpha = 0
        else:
            alpha = math.degrees(math.asin(self.robot.catch_distance / float(displacement)))
        beta = 90 - alpha
        print(alpha, beta)

        if math.sin(math.radians(alpha)) == 0:
            distance_to_move = 15  # something is obviously wrong so we have to move a bit
        else:
            distance_to_move = math.sin(math.radians(beta)) * self.robot.catch_distance / math.sin(math.radians(alpha))

        comms.move(distance_to_move)


class GrabBall(Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball),
                     lambda w, r: r.catcher == 'OPEN']

    def perform(self, comms):
        comms.close_grabbers()


class TurnToCatchPoint(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(utils.defender_get_rotation_to_catch_point(Vector(self.robot.x, self.robot.y, self.robot.angle, 0), Vector(x, y, 0, 0), self.robot.catch_distance))


class Shoot(Action):
    preconditions = [lambda w, r: r.has_ball(w.ball),
                     lambda w, r: utils.can_score(w, r, w.their_goal)]

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
    preconditions = [lambda w, r: r.on_goal_arc(w.our_goal)]

    def perform(self, comms):
        raise NotImplementedError


class TurnToOpposingAttacker(Action):
    '''
    Turn to face opponent's attacker
    '''
    preconditions = [lambda w, r: r.aligned_on_goal_arc(w.robot_in_possession)]

    def perform(self, comms):
        raise NotImplementedError


class OpenGrabbersForOpponentShot(Action):
    '''
    Open grabbers to block oponnent's shot
    '''
    def rotation_precondition(self, w, r):
        possessing = w.robot_in_posession
        rotation = utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(possessing.x, possessing.y, 0, 0), r.catch_distance)
        return abs(rotation) < FACING_ROTATION_THRESHOLD

    preconditions = [lambda w, r: r.aligned_on_goal_arc(w.robot_in_possession),
                     rotation_precondition]

    def perform(self, comms):
        raise NotImplementedError


class TurnToScoreZone(Action):
    '''
    Turn with ball to prepare pass to attacker's score zone
    '''
    preconditions = [lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        raise NotImplementedError


class KickToScoreZone(Action):
    '''
    Pass ball to our attacker's score zone
    '''
    preconditions = [lambda w, r: abs(r=utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.score_zone.x, w.score_zone.y, 0, 0), r.catch_distance)) < FACING_ROTATION_THRESHOLD,
                     lambda w, r: r.has_ball(w.ball)]

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
    preconditions = [lambda w, r: are_equivalent_positions(r, r.tactical_position)]

    def perform(self, comms):
        raise NotImplementedError
