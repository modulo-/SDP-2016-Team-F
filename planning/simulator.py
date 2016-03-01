
import getopt
import sys
import cocos
import cocos.actions as ac
import math

from threading import Timer
from position import Vector
from world import World
from planner import DefencePlanner
from comms import CommsManager

import logging


class Test:

    def __init__(self, scene):
        self.scene = scene

    def initialise(self, initial_state):
        self.our_defender = None
        self.our_attacker = None
        if initial_state['our_defender']:
            self.our_defender = self.scene.add_robot("defender", initial_state['our_defender'])
        if initial_state['our_attacker']:
            self.our_attacker = self.scene.add_robot("attacker", initial_state['our_attacker'])
        self.ball = self.scene.add_ball(initial_state['ball'])
        self.p = DefencePlanner(comms=SimulatorComms(self.our_defender, self.ball, self.wait_and_next_step))
        self.p.set_task(initial_state['task'])
        self.w = World('left', 0)
        self.w.our_defender._receiving_area = {'width': 40, 'height': 50, 'front_offset': 20}
        self.w.our_defender._catch_distance = 32
        self.steps = 0
        self.max_steps = initial_state['max_steps']

    def run(self, initial_state, test_name):
        logging.info(">> " + test_name + " started")
        self.initialise(initial_state)
        self.step()

    def step(self):
        print("\nStep: " + str(self.steps))
        if self.steps >= self.max_steps:
            print "Finished"
            return

        # update & act
        self.w.update_positions(our_attacker=self.our_attacker.get_vec() if self.our_attacker is not None else None, our_defender=self.our_defender.get_vec() if self.our_defender is not None else None, ball=self.ball.get_vec())
        self.p.plan_and_act(self.w)

        self.steps += 1

    def wait_and_next_step(self, delay):
        print("Waiting for " + str(delay) + " seconds...")
        t = Timer(delay, self.step)
        t.start()

    def test1(self):
        initial_state = {
            'task': 'move-grab',
            'max_steps': 2,
            'our_defender': Vector(100, 100, math.radians(270), 0),
            'our_attacker': None,
            'ball': Vector(200, 170, 0, 0),
        }

        self.run(initial_state, "Test 1")

    def test2(self):
        initial_state = {
            'task': 'move-grab',
            'max_steps': 2,
            'our_defender': Vector(100, 100, math.radians(315), 0),
            'our_attacker': None,
            'ball': Vector(200, 170, 0, 0),
        }

        self.run(initial_state, "Test 2")

    def test3(self):
        initial_state = {
            'task': 'move-grab',
            'max_steps': 2,
            'our_defender': Vector(250, 220, math.radians(200), 0),
            'our_attacker': None,
            'ball': Vector(200, 170, 0, 0),
        }

        self.run(initial_state, "Test 3")


class Scene(cocos.layer.ColorLayer):

    def __init__(self, test):
        super(Scene, self).__init__(100, 255, 100, 255)
        t = Test(self)

        if(test == "1"):
            t.test1()
        elif(test == "2"):
            t.test2()
        elif(test == "3"):
            t.test3()
        elif(test == "m31"):
            t.testm3_1()
        else:
            print("NO TEST ASSOCIATED!")

    def add_robot(self, vec):
        robot = Robot(pos=[vec.x, vec.y], rotation_radians=vec.angle)
        self.add(robot)
        return robot

    def add_ball(self, vec):
        ball = Ball(pos=[vec.x, vec.y])
        self.add(ball)
        return ball


class SimulatorComms(CommsManager):

    def __init__(self, robot, ball, wait_and_next_step):
        self.robot = robot
        self.ball = ball
        self.wait_and_next_step = wait_and_next_step
        super(SimulatorComms, self).__init__(0)

    def move(self, d):
        super(SimulatorComms, self).move(d)
        print("Robot move distance: {0}".format(d))

        rotation = self.robot.rotation + 90
        dx = d * math.sin(math.radians(rotation))
        dy = d * math.cos(math.radians(rotation))
        # print("Robot dxdy: {0} {1}".format(dx, dy))
        delay = self.robot.move_to(
            self.robot.position[0] + dx,
            self.robot.position[1] + dy)
        self.wait_and_next_step(delay)

    def turn(self, angle):
        '''
        positive angle - counter-clockwise rotation
        negative angle - clockwise rotation
        angle 0 means turned right
        '''
        print("Robot rotation: {0}".format(self.robot.rotation))
        super(SimulatorComms, self).turn(angle)
        delay = self.robot.rotate_by(angle)
        self.wait_and_next_step(delay)

    def close_grabbers(self):
        super(SimulatorComms, self).close_grabbers()
        # TODO Fix assumption that we can grab ball
        self.ball.grab(self.robot)

        self.wait_and_next_step(1)

    def release_grabbers(self):
        super(SimulatorComms, self).release_grabbers()
        self.ball.release()

        self.wait_and_next_step(1)


class Sprite (cocos.sprite.Sprite):

    def __init__(self, sprite, pos):
        super(Sprite, self).__init__(sprite)
        self.velocity = (0, 0)
        self.scale = 1
        self.set_position(pos[0], pos[1])
        self.rotation = 0

    def set_position(self, x, y):
        self.position = (x, y)

    def move_to(self, x, y):
        self.do(ac.MoveTo((x, y), duration=self._movement_speed))
        print("Moving from: {0}".format(self.position))
        return self._movement_speed

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


class Robot(Sprite):

    def __init__(self, pos, rotation_radians):
        super(Robot, self).__init__('res/robot_side.png', pos)
        self._movement_speed = 2
        self._rotation_speed = 1

        if rotation_radians < 0:
            rotation_radians += math.pi * 2
        self.rotation = math.degrees(rotation_radians)


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
    def __init__(self, width, height, verbose, test):

        logging.basicConfig(level=logging.INFO, format="\033[95m\r%(asctime)s - %(levelname)s - %(message)s\033[0m")

        # director init takes the same arguments as pyglet.window
        cocos.director.director.init(width=width, height=height, resizable=False)
        world = Scene(test)

        # A scene that contains the layer hello_layer
        main_scene = cocos.scene.Scene(world)

        # And now, start the application, starting with main_scene
        cocos.director.director.run(main_scene)


def usage():
    print("Just use it!")


def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], "hvrt:", ["help", "verbose", "run", "test="])
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err)  # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    verbose = False
    for o, a in opts:
        if o == "-v":
            verbose = True
        elif o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-r", "--run"):
            Environment(width=600, height=400, verbose=verbose)
        elif o in ("-t", "--test"):
            Environment(width=600, height=400, verbose=verbose, test=a)
        else:
            assert False, "unhandled option"


if __name__ == "__main__":
    main()
