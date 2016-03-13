
import getopt
import sys
import cocos
import cocos.actions as ac
import math

from threading import Timer
from position import Vector
from world import World
from planner import DefencePlanner, AttackPlanner
from comms import CommsManager

import logging

class Game:

    def __init__(self, scene):
        self.scene = scene
        self.our_defender = self.scene.add_robot("defender", Vector(20, 200, math.radians(90), 0))
        self.our_attacker = self.scene.add_robot("attacker", Vector(150, 200, math.radians(90), 0))
        self.their_defender = self.scene.add_robot("defender", Vector(580, 200, math.radians(270), 0))
        self.their_attacker = self.scene.add_robot("attacker", Vector(450, 200, math.radians(270), 0))
        self.ball = self.scene.add_ball(Vector(300, 200, 0, 0))
        self.odp = DefencePlanner(comms=SimulatorComms(self.our_defender, self.ball))
        self.oap = AttackPlanner(comms=SimulatorComms(self.our_attacker, self.ball))
        self.tdp = DefencePlanner(comms=SimulatorComms(self.their_defender, self.ball))
        self.tap = AttackPlanner(comms=SimulatorComms(self.their_attacker, self.ball))
        self.odp.set_task('game')
        self.oap.set_task('game')
        self.tdp.set_task('game')
        self.tap.set_task('game')
        self.world = World('left', 0)

    def run(self):
        self.step()

    def step(self):
        self.world.update_positions(our_attacker=self.our_attacker.get_vec(),
                                        our_defender=self.our_defender.get_vec(),
                                        their_robot_0=self.their_attacker.get_vec(),
                                        their_robot_1=self.their_defender.get_vec(),
                                        ball=self.ball.get_vec())
        Timer(max([self.odp.plan_and_act(self.world),
                   self.oap.plan_and_act(self.world),
                   self.tdp.plan_and_act(self.world),
                   self.tap.plan_and_act(self.world)
                   ]), self.step).start()

class Scene(cocos.layer.ColorLayer):

    def __init__(self):
        super(Scene, self).__init__(100, 255, 100, 255)
        g = Game(self)
        g.run()

    def add_robot(self, robot_type, vec):
        if (robot_type == "defender"):
            robot = Defender(pos=[vec.x, vec.y], rotation_radians=vec.angle)
        elif (robot_type == "attacker"):
            robot = Attacker(pos=[vec.x, vec.y], rotation_radians=vec.angle)

        self.add(robot)
        return robot

    def add_ball(self, vec):
        ball = Ball(pos=[vec.x, vec.y])
        self.add(ball)
        return ball


class SimulatorComms(CommsManager):

    def __init__(self, robot, ball):
        self.robot = robot
        self.ball = ball
        super(SimulatorComms, self).__init__(0)

    def move(self, d):
        super(SimulatorComms, self).move(d)
        print("Robot move distance: {0}".format(d))

        rotation = self.robot.rotation + 90
        dx = d * math.sin(math.radians(rotation))
        dy = d * math.cos(math.radians(rotation))

        delay = self.robot.move_to(
            self.robot.position[0] + dx,
            self.robot.position[1] + dy)
        return delay

    def turn(self, angle):
        '''
        positive angle - counter-clockwise rotation
        negative angle - clockwise rotation
        angle 0 means turned right
        '''
        print("Robot rotation: {0}".format(self.robot.rotation))
        super(SimulatorComms, self).turn(angle)
        delay = self.robot.rotate_by(angle)
        return delay

    def close_grabbers(self):
        super(SimulatorComms, self).close_grabbers()
        # TODO Fix assumption that we can grab ball
        self.ball.grab(self.robot)

        return 1

    def release_grabbers(self):
        super(SimulatorComms, self).release_grabbers()
        self.ball.release()

        return 1


class Sprite (cocos.sprite.Sprite):

    def __init__(self, sprite, pos):
        super(Sprite, self).__init__(sprite)
        self.velocity = (0, 0)
        self.scale = 1
        self.set_position(pos[0], pos[1])
        self.rotation = 0

    def set_position(self, x, y):
        self.position = (x, y)

    def rotate_by(self, radians):
        '''
        positive angle - counter-clockwise rotation
        negative angle - clockwise rotation
        angle 0 means turned right
        '''
        angle = math.degrees(radians)
        self.do(ac.RotateBy(angle, self._rotation_speed))
        return self._rotation_speed

    def get_vec(self):
        return Vector(self.x, self.y, math.radians(self.rotation), 0)


class Defender(Sprite):

    def __init__(self, pos, rotation_radians):
        super(Defender, self).__init__('res/robot_side.png', pos)
        self._movement_speed = 3
        self._rotation_speed = 2
        self.rotation = math.degrees(rotation_radians % (math.pi * 2))

    def move_to(self, x, y):
        self.do(ac.MoveTo((x, y), duration=self._movement_speed))
        print("Moving from: {0}".format(self.position))
        return self._movement_speed


class Attacker(Sprite):

    def __init__(self, pos, rotation_radians):
        super(Attacker, self).__init__('res/robot_side.png', pos)
        self._movement_speed = 3
        self._rotation_speed = 2
        self.rotation = math.degrees(rotation_radians % (math.pi * 2))

    def move_to(self, x, y):
        self.do(ac.MoveTo((x, y), duration=self._movement_speed))
        print("Moving from: {0}".format(self.position))
        return self._movement_speed


class Ball(Sprite):

    def __init__(self, pos):
        super(Ball, self).__init__('res/ball.png', pos)
        self._movement_speed = 15

    def grab(self, robot):
        pass

    def release(self):
        self._anchor_x = 0
        self._anchor_y = 0


class Environment():
    def __init__(self, width, height, verbose):

        if verbose:
            logging.basicConfig(level=logging.DEBUG, format="\033[95m\r%(asctime)s - %(levelname)s - %(message)s\033[0m")
        else:
            logging.basicConfig(level=logging.INFO, format="\033[95m\r%(asctime)s - %(levelname)s - %(message)s\033[0m")

        # director init takes the same arguments as pyglet.window
        cocos.director.director.init(width=width, height=height, resizable=False)
        world = Scene()

        # A scene that contains the layer hello_layer
        main_scene = cocos.scene.Scene(world)

        # And now, start the application, starting with main_scene
        cocos.director.director.run(main_scene)


def usage():
    print("Just use it!")


def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "v:", ["verbose"])
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err)  # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    verbose = False
    for o, a in opts:
        if o == "-v":
            verbose = True
        else:
            assert False, "unhandled option"

    Environment(width=600, height=400, verbose=verbose)


if __name__ == "__main__":
    main()
