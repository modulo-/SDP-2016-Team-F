import utils
import math

from position import Vector

# TODO
ROTATION_THRESHOLD = 0.35
FACING_ROTATION_THRESHOLD = 0.35
DISTANCE_THRESHOLD = 5


def are_equivalent_positions(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2) < DISTANCE_THRESHOLD


class Goal(object):
    '''
    Base class for goals
    '''
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot

    # Return the next action necesary to achieve the goal
    def generate_action(self):
        for a in self.actions:
            if a.is_possible():
                return a
        return None


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
                     lambda w, r: abs(utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.ball.x, w.ball.y, 0, 0))) < ROTATION_THRESHOLD]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)

        # TODO grabbing area size
        grabber_size = 30
        comms.move(d - grabber_size)


class GrabBall(Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball),
                     lambda w, r: r.catcher == 'OPEN']

    def perform(self, comms):
        comms.close_grabbers()


class TurnToCatchPoint(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(utils.defender_get_rotation_to_catch_point(Vector(self.robot.x, self.robot.y, self.robot.angle, 0), Vector(x, y, 0, 0)))


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
        rotation = utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(possessing.x, possessing.y, 0, 0))
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
    preconditions = [lambda w, r: abs(r=utils.defender_get_rotation_to_catch_point(Vector(r.x, r.y, r.angle, 0), Vector(w.score_zone.x, w.score_zone.y, 0, 0))) < FACING_ROTATION_THRESHOLD,
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
