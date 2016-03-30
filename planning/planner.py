import models_attacker as attacker
import models_defender as defender
import utils
import math

from comms import CommsManager
from logging import info
from models_common import DEFAULT_DELAY


class Planner (object):
    '''
    Takes the world and creates a plan and actuates it.
    '''

    def __init__(self, comms=CommsManager(0)):
        self.comms = comms
        self.previous_action = None
        self.current_task = 'reactive-grab'
        self.grabber_state = 'CLOSED'

    def set_task(self, task):
        self.current_task = task

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''
        raise NotImplementedError

    def actuate(self, action):
        '''Perform actions'''
        if action is None:
            return 1
        delay = action.perform(self.comms)
        return delay

    def plan_and_act(self, world):
        '''
        Make plans for each robot, perform them and return delay
        '''
        robot = self.robot(world)
        goal = self.get_goal(world, robot)
        if goal is None:
            info("Planner has no goal")
            return DEFAULT_DELAY
        action = goal.generate_action()
        if action != self.previous_action:
            action = action
        self.previous_action = action
        delay = self.actuate(action)
        if not delay:
            try:
                delay = action.get_delay()
            except AttributeError:
                delay = DEFAULT_DELAY
        print ("GONNA WAIT >>>: " + str(delay))
        return delay


class AttackPlanner(Planner):
    '''
    Planner for attacking robot
    '''

    def actuate(self, action):
        Planner.actuate(self, action)
        return action.get_delay()

    def robot(self, world):
        return world.our_attacker

    def plan_and_act(self, world):
        return super(AttackPlanner, self).plan_and_act(world)

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''
        '''        if robot.has_ball(world.ball):
            return attacker.Score(world, robot)
        else:
            return attacker.GetBall(world, robot)'''
        if self.current_task == 'move-grab':
            return attacker.GetBall(world, robot)
        elif self.current_task == 'turn-move-grab':
            return attacker.GetBall(world, robot)
        elif self.current_task == 'turn-shoot':
            if robot.has_ball(world.ball):
                return attacker.Score(world, robot)
            else:
                info("Trying to turn-shoot without ball. Using GetBall instead.")
                return attacker.GetBall(world, robot)
        elif self.current_task == 'receive-pass':
            if world.our_defender.has_ball(world.ball):
                return attacker.AttackPosition(world, robot)
            else:
                return attacker.GetBall(world, robot)
        elif self.current_task == 'receive-turn-pass':
            if world.their_robots[0].has_ball(world.ball) or world.their_robots[1].has_ball(world.ball):
                return attacker.AttackPosition(world, robot)
            elif robot.has_ball(world.ball):
                return attacker.AttackerPass(world, robot)
            else:
                return attacker.GetBall(world, robot)
        elif self.current_task == 'intercept':
            return attacker.AttackerBlock(world, robot)
        elif self.current_task == 'score-zone':
            return attacker.AttackPosition(world, robot)
        elif world.game_state == 'normal-play':
            if world.our_attacker.has_ball(world.ball):
                info("Attacker has ball so trying to score")
                return attacker.Score(world, robot)
            elif world.our_defender.has_ball(world.ball):
                info("Defender has ball so going to attack position")
                return attacker.AttackPosition(world, robot)
            elif any([r.has_ball(world.ball) for r in world.their_attackers]):
                info("Opponent in our half has ball so going to score position")
                return attacker.AttackPosition(world, robot)
            elif any([r.has_ball(world.ball) for r in world.their_defenders]):
                info("Opponent in their half has ball so blocking")
                return attacker.AttackerBlock(world, robot)
            elif world.is_possible_position(world.our_attacker, world.ball.x, world.ball.y):
                info("Ball is in possible position so getting ball")
                return attacker.GetBall(world, robot)
            else:
                info("All else failed so going to attack position")
                return attacker.AttackPosition(world, robot)
        elif self.current_task == 'test-obstacle':
            print(math.degrees(utils.get_avoiding_angle_to_point(world,
                                                    world.our_attacker.vector,
                                                    world.ball.vector)))
            from position import Vector
            print(utils.find_obstacle(world, robot.vector, world.ball.vector))
            return attacker.GetBall(world, robot)


class DefencePlanner(Planner):
    '''
    Planner for defending robot
    '''

    def robot(self, world):
        return world.our_defender

    def get_goal(self, world, robot):
        '''
        Selects a goal for robot
        '''
        if robot.penalty:
            return None
        elif self.current_task == 'game' and world.game_state is not None:
            utils.defender_rotation_to_defend_point(robot, world.ball, world.our_goal.vector, defender.GOAL_RADIUS)
            if robot.has_ball(world.ball):
                info("Defender goal choice: Pass the ball")
                return defender.Pass(world, robot)
            elif not utils.ball_heading_to_our_goal(world) and utils.defender_should_grab_ball(world):
                info("Defender goal choice: Retrieve the ball")
                return defender.GetBall(world, robot)
            else:
                info("Defender goal choice: Do the wiggle dance!")
                return defender.Defend(world, robot)
        elif self.current_task == 'oldgame' and world.game_state is not None:
            if robot.has_ball(world.ball):
                info("Defender goal choice: kick the ball")
                return defender.Pass(world, robot)
            elif utils.ball_heading_to_our_goal(world) and world.in_our_half(world.ball):
                info("Defender goal choice: Intercept")
                return defender.ReactiveGrabGoal(world, robot)
            elif utils.ball_is_static(world) and world.in_our_half(world.ball):
                ourdist = math.hypot(world.ball.x - robot.x, world.ball.y - robot.y)
                oppdists = [math.hypot(world.ball.x - r.x, world.ball.y - r.y)
                            for r in world.their_robots if not r.is_missing()]
                if (len(oppdists) == 0 or ourdist < min(oppdists)) and world.game_state == 'normal-play':
                    info("Defender goal choice: Retrieve ball")
                    return defender.GetBall(world, robot)
                else:
                    info("Defender goal choice: Block")
                    return defender.Block(world, robot)
            else:
                info("Defender goal choice: Return to defence area")
                return defender.ReturnToDefenceArea(world, robot)
        elif self.current_task == 'move-grab':
            return defender.GetBall(world, robot)
        elif self.current_task == 'defend':
            return defender.Defend(world, robot)
        elif self.current_task == 'reactive-grab':
            return defender.ReactiveGrabGoal(world, robot)
        elif self.current_task == 'm1':
            return defender.ReceivingPass(world, robot)
        elif self.current_task == 'm2':
            return defender.ReceiveAndPass(world, robot)
        elif self.current_task == 'm31':
            return defender.InterceptPass(world, robot)
        elif self.current_task == 'm32':
            return defender.InterceptGoal(world, robot)
