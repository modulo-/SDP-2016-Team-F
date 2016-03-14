import models_attacker as attacker
import models_defender as defender

from comms import CommsManager
from logging import info, error
from models_common import DEFAULT_DELAY
import utils
import math


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
        if isinstance(action, attacker.GrabBall):
            info("Did grab")
            self.grabber_state = 'CLOSED'
            self.has_grabbed = True
        elif isinstance(action, attacker.OpenGrabbers):
            info("Did open")
            self.grabber_state = 'OPEN'
            self.has_grabbed = False
        return Planner.actuate(self, action)

    def robot(self, world):
        return world.our_attacker

    def plan_and_act(self, world):
        world.our_attacker.catcher = self.grabber_state
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
        elif self.current_task == 'game':
            if world.our_attacker.has_ball(world.ball):
                return attacker.Score(world, robot)
            elif world.our_defender.has_ball(world.ball):
                return attacker.AttackPosition(world, robot)
            elif any([r.has_ball(world.ball) for r in world.their_attackers]):
                return attacker.AttackPosition(world, robot)
            elif any([r.has_ball(world.ball) for r in world.their_defenders]):
                return attacker.AttackerBlock(world, robot)
            else:
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
        elif self.current_task == 'play' and world.game_state != None:
            if robot.has_ball(world.ball):
                info("Defender goal choice: kick the ball")
                return defender.Pass(world, robot)
            elif utils.ball_heading_to_our_goal(world) and world.in_our_half(world.ball):
                info("Defender goal choice: Intercept")
                return defender.ReactiveGrabGoal(world, robot)
            elif utils.ball_is_static(world) and world.in_our_half(world.ball):
                ourdist = math.hypot(world.ball.x - robot.x, world.ball.y -
                        robot.y)
                oppdists = [math.hypot(world.ball.x - r.x, world.ball.y - r.y)
                        for r in world.their_robots if not r.is_missing()]
                if ourdist < min(oppdists) and world.game_state == 'play':
                    info("Defender goal choice: Retrieve ball")
                    return defender.GetBall(world, robot)
                else:
                    info("Defender goal choice: Block")
                    return defender.Block(world, robot)
            else:
                info("Defender goal choice: Return to defence area")
                # TODO
                pass
        elif self.current_task == 'move-grab':
            return defender.GetBall(world, robot)
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
