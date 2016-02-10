import utils
import math

# TODO
ROTATION_THRESHOLD = 0.1


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


class GoToStaticBall_normal(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(r.get_rotation_to_point(w.ball.x, w.ball.y)) < ROTATION_THRESHOLD]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)
        # TODO grabbing area size
        grabber_size = 50
        proportion = (d - grabber_size) / d
        target_pos = (self.robot.x + dx * proportion, self.robot.y + dy * proportion, self.robot.angle)
        comms.move(robot_pos=(self.robot.x, self.robot.y, self.robot.angle), target_pos=target_pos, distance=(d - grabber_size))


class GoToStaticBall(Action):
    preconditions = [lambda w, r: utils.ball_is_static(w),
                     lambda w, r: abs(r.get_rotation_to_point(w.ball.x, w.ball.y)) < ROTATION_THRESHOLD]

    def perform(self, comms):
        dx = self.world.ball.x - self.robot.x
        dy = self.world.ball.y - self.robot.y
        d = math.sqrt(dx**2 + dy**2)

        pdx = -dy
        pdy = dx
        pd = math.sqrt(pdx**2 + pdy**2)
        norm_pvec = (float(pdx) / float(pd), float(pdy) / float(pd))
        print("normalized vector: {0}".format(norm_pvec))

        grabber_distance = 60

        target_pos = (self.world.ball.x + norm_pvec[0] * grabber_distance, self.world.ball.y + norm_pvec[1] * grabber_distance, self.robot.angle)
        comms.move(robot_pos=(self.robot.x, self.robot.y, self.robot.angle), target_pos=target_pos, distance=d)


class GrabBall(Action):
    preconditions = [lambda w, r: r.can_catch_ball(w.ball)]

    def perform(self, comms):
        comms.close_grabbers()


class TurnToGoal(Action):
    preconditions = [lambda w, r: r.has_ball(w.ball)]

    def perform(self, comms):
        # TODO find best point to shoot to
        x = self.world.their_goal.x + self.world.their_goal.width / 2
        y = self.world.their_goal.y
        comms.turn(self.robot.get_rotation_to_point(x, y))


class TurnToBall(Action):
    def perform(self, comms):
        x = self.world.ball.x
        y = self.world.ball.y
        comms.turn(self.robot.get_rotation_to_point(x, y))


class Shoot(Action):
    preconditions = [lambda w, r: r.has_ball(w.ball),
                     lambda w, r: utils.can_score(w, r, w.their_goal)]

    def perform(self, comms):
        comms.kick_full_power()
